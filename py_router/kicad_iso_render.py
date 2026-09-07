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
  it chose itself. ``REQUEST_OVERSCAN`` exists so that letterbox downscales
  (sharp) rather than upscales (blurry).
* **Cost is ~2-4 s at ``basic``, and CONTENTION matters more than the board.**
  Across tigard, lvds, ulx3s (225 models) and glasgow_revC (224), 3 reps each,
  a quiet serial pass ran 1.4-2.7 s and spread under 2x, with glasgow
  consistently slowest; running four at once -- what ``--iso-jobs 4`` actually
  does -- ran 1.9-4.2 s. ``--quality high`` is about 3x that, 5.0-7.5 s, which
  is why ``basic`` is the default. So a render per FRAME is out of the question
  and a render per chain STEP is affordable -- and eight of them run about 2.4x
  faster over six workers than serially (measured 24.0 s -> 10.2 s).
* **Component bodies are board-dependent, and their absence is silent.**
  ``kicad_files/tigard.kicad_pcb`` renders as a BARE BOARD -- pads, mask,
  silkscreen, no parts. Its 84 ``(model ...)`` references are 81
  ``${KISYS3DMOD}`` + 3 ``${KIPRJMOD}``, and 82 of them name a ``.wrl``, against
  a KiCad 10 tree that ships ``.step`` only; passing ``-D KISYS3DMOD=<dir>``
  does not fix it. ``kicad_files/lvds_converter_dualclk`` renders with full
  bodies. kicad-cli says nothing either way, so ``resolve_models`` counts what
  is actually on disk and the caller captions the answer instead of shipping an
  empty green rectangle that reads as a bug.
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

#: Ask for MORE than the box, because the letterbox throws most of it away.
#: kicad-cli frames the board with a wide margin and the caller crops to the
#: alpha box (``movie_panels._alpha_crop``), so the pixels that survive are only
#: the board -- measured around 0.6 of the canvas on each axis. At the old 1.15
#: that left the cropped board being UPSCALED into the panel, which is the blur
#: this constant exists to prevent; 1.8 keeps it a downscale. Renders cost
#: roughly the same at either size (measured), so the margin is close to free.
REQUEST_OVERSCAN = 1.8

#: Model-directory variables. Census over the TRACKED boards in kicad_files/:
#: KICAD6_3DMODEL_DIR 537, KISYS3DMOD 498 (the KiCad 5/6 spelling), KICAD9 84,
#: KIPRJMOD 47, KICAD10 33, KICAD8 23, KICAD_USER_DIR 1.
#:
#: `KICAD7_3DMODEL_DIR` occurs ZERO times and is here defensively -- KiCad 7
#: exists and boards from it will arrive eventually. Called out because the
#: comment used to say all of these were "measured over kicad_files/", which
#: implied every one had been observed.
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
    # A BARE RELATIVE model path is relative to the PROJECT -- it is
    # `${KIPRJMOD}/...` with the variable left off, and that is how KiCad reads
    # it. Resolving it against os.getcwd() instead made the answer depend on
    # the CALLER'S DIRECTORY: whether a reference resolved was decided by what
    # happened to sit beside the shell, and anything it found that way was a
    # file kicad-cli would never load -- a count under the picture that need
    # not describe the picture.
    #
    # 10 of the 22 TRACKED boards in kicad_files/ carry at least one bare
    # relative reference, so this is the common case, not a corner. (An earlier
    # comment here said "13 of 26" and illustrated it with lvds reporting 13/15
    # from one directory and 10/15 from another. Both were measured in a dirty
    # tree: 26 counts four generated boards that are gitignored, and lvds's
    # three bare refs name .stp files that exist nowhere in the repo, so no cwd
    # produces 13. The defect was real and the fix is unchanged; the numbers
    # were not reproducible and are replaced with ones that are.)
    proj = dirs.get('KIPRJMOD') or os.path.dirname(os.path.abspath(board_path))
    for raw in _MODEL_RE.findall(txt):
        out['total'] += 1
        path = _VAR_RE.sub(lambda m: dirs.get(m.group(1), _UNRESOLVED), raw)
        if _UNRESOLVED in path:
            out['unresolved_var'] += 1
            if not out['example']:
                out['example'] = raw
            continue
        path = path.replace('\\', '/')
        if not os.path.isabs(path):
            path = os.path.join(proj, path).replace('\\', '/')
        if os.path.isfile(path):
            out['found'] += 1
            continue
        stem = os.path.splitext(path)[0]
        if any(os.path.isfile(stem + e) for e in _MODEL_EXTS):
            out['other_ext'] += 1
        if not out['example']:
            out['example'] = raw
    return out


#: Below this share of models resolved, the render is bodies-in-name-only and
#: the caption says so. NOT `found == 0`: gating on exactly zero meant four
#: corpus boards that render essentially bare got no warning at all --
#: kit-dev-coldfire-xilinx_5213 at 1/160, orangecrab_ext_pll 5/148,
#: splitflap_driver 3/58, watchy 7/75 -- because one resolved model out of 160
#: is not a populated board, and the stale-reference explanation stopped
#: applying one model above zero.
MOSTLY_BARE_FRACTION = 0.25


def models_note(models):
    """One human clause for a caption. Never claims more than it counted."""
    if not models or not models.get('total'):
        return 'no 3D models referenced'
    t, f = models['total'], models['found']
    note = '3D models %d/%d' % (f, t)
    if f == 0:
        note += ' -- BARE BOARD, no component bodies'
    elif f < t * MOSTLY_BARE_FRACTION:
        note += ' -- MOSTLY BARE'
    else:
        return note
    # The REASON, at both severities. A miss whose same-stem twin exists under
    # another extension is a stale reference in the board, not a missing
    # install, and the two have different fixes.
    if models.get('other_ext'):
        note += ' (%d exist under another extension)' % models['other_ext']
    elif models.get('unresolved_var'):
        note += ' (%d paths use an undefined variable)' % models['unresolved_var']
    return note


def render_iso(board_path, out_png, cli, width, height, rotate=ISO_ROTATE,
               quality='basic', floor=False, perspective=False, zoom=None,
               timeout=ISO_RENDER_HANG_GUARD_S):
    """One ``kicad-cli pcb render``. ``(png_path, error)``; error ``''`` on success.

    The PNG keeps kicad-cli's default TRANSPARENT background on purpose: the
    caller alpha-composites it onto its own panel colour, whereas
    ``--background opaque`` would paste KiCad's light viewer background into a
    dark movie.

    argv is built as a LIST and never a string, and ``shell`` is never true:
    ``--rotate -45,0,45`` has to arrive as one argument, and on Windows a shell
    would be free to reinterpret it.

    There is no ``define=`` emitting ``-D VAR=path``. One existed, unused by
    every caller and every test, for the 3D-model story below -- and the thing
    it was written for was MEASURED NOT TO WORK: ``-D KISYS3DMOD=<a real
    directory>`` did not make tigard's 82 ``.wrl`` references resolve, because
    a KiCad 10 tree ships ``.step`` and the substitution finds nothing at the
    substituted path either. Shipping the parameter would have implied a
    workaround exists.
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
    # Exit 0 and a file on disk does NOT establish a decodable image. A
    # zero-byte or truncated write (a full disk, a killed child) used to be
    # reported as unqualified success while the composer quietly drew
    # "could not read the render" into the panel -- three broken panels under a
    # status line saying everything worked, which is the one outcome the named
    # degrade states exist to prevent. Verify here, where a reason can still be
    # returned.
    try:
        from PIL import Image
        with Image.open(out_png) as probe:
            probe.verify()
    except ImportError:
        pass                    # no Pillow: the composer cannot draw anyway
    except Exception as exc:                                    # noqa: BLE001
        return None, ('kicad-cli exited 0 but wrote an unreadable PNG (%s, '
                      '%d bytes)' % (exc, os.path.getsize(out_png)
                                     if os.path.isfile(out_png) else 0))
    return out_png, ''


def render_many(jobs, cli, workers=None, **kw):
    """``{key: (png|None, error)}`` for ``jobs = [(key, board, out_png, rotate)]``.

    Threads, not processes: the unit of work is a subprocess, so the GIL is
    released and there is nothing to pickle. Capped at 4 because kicad-cli is
    itself multithreaded and oversubscribing past that stopped buying anything
    (measured: 8 renders, 24.0 s serial vs 10.2 s over six workers, ~2.4x).

    Results are keyed, never appended in completion order, so the composed movie
    is identical at any worker count. A test asserts exactly that, and also that
    more than one thread really ran -- otherwise the determinism claim could be
    satisfied by silently serialising.

    **That guarantee is the default quality's, not kicad-cli's.** Measured, two
    serial renders of the same job on two boards:

        --quality basic : bytes identical, pixels identical  (2 of 2 boards)
        --quality high  : bytes DIFFER, pixels DIFFER        (2 of 2 boards)

    So at `basic` -- the default, and what the panel uses unless asked otherwise
    -- a render is reproducible and the movie is too. At `high` kicad-cli is not
    reproducible against ITSELF, run to run, on one thread; nothing here can make
    it so, and `--iso-jobs` is not what changes the answer. Said out loud because
    an earlier version of this comment had it backwards, claiming byte
    instability at basic on the strength of one noisy sample.
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
                    help='basic measured 1.4-2.7 s serial and 1.9-4.2 s four '
                         'at once; high measured 5.0-7.5 s on the same four '
                         'boards, about 3x (default: basic)')
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
