#!/usr/bin/env python3
"""#887: the half that needs a REAL kicad-cli. Self-skips without one.

`tests/test_887_iso_models.py` grades everything that can be graded with
kicad-cli monkeypatched, on any machine. This file grades the
three things only a real render can answer, and it exists because each of them
was measured once and would otherwise be a comment nobody re-checks:

  * kicad-cli does NOT return the size you asked for;
  * parallel renders are identical to serial ones, and really do run in
    parallel -- and that reproducibility belongs to the DEFAULT quality, not to
    kicad-cli: at --quality high it is not reproducible against itself;
  * a board whose 3D models do not resolve still renders, and SAYS it is bare.
"""
import atexit
import os
import shutil
import sys
import tempfile

RUN_ALL_TIMEOUT = 900

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

try:
    from PIL import Image
except ImportError:
    print('SKIP: Pillow is not installed')
    sys.exit(77)

import kicad_iso_render as kir                             # noqa: E402

CLI, WHY = kir.resolve_cli()
if not CLI:
    print('SKIP: no kicad-cli on this machine (%s). '
          'tests/test_887_iso_models.py carries the regression.' % WHY)
    sys.exit(77)

def _stage(name, into):
    """Copy `name` plus every sibling sharing its stem into `into`.

    Staging is NOT tidiness: `kicad-cli pcb render` REWRITES the board's
    sibling `.kicad_prl`. Measured -- one render against
    kicad_files/routed_output.kicad_pcb changed the md5 of
    kicad_files/routed_output.kicad_prl, a TRACKED 155-line file, replacing it
    with a 5-line one. Rendering kicad_files/ in place therefore leaves the
    working tree dirty, and a later `git add -A` commits the damage; that is
    exactly what happened once while this file was being written. Boards with
    no sibling project (lvds, tigard) are unaffected today -- measured, `git
    status` stays clean -- but they are staged too, so that adding a project
    to one of them later cannot silently re-open this.
    """
    src = os.path.join(ROOT, 'kicad_files', name)
    stem = os.path.splitext(os.path.basename(src))[0]
    for sib in os.listdir(os.path.dirname(src)):
        if os.path.splitext(sib)[0] == stem:
            shutil.copy2(os.path.join(os.path.dirname(src), sib),
                         os.path.join(into, sib))
    return os.path.join(into, os.path.basename(src))


_STAGE = tempfile.mkdtemp(prefix='krt887_boards_')
atexit.register(shutil.rmtree, _STAGE, True)

LVDS = _stage('lvds_converter_dualclk.kicad_pcb', _STAGE)
#: A second, DIFFERENT board, so the chain has two beats. Tracked -- the
#: first choice here was qfn_fanned_out.kicad_pcb, which is GITIGNORED
#: (.gitignore:44) and generated on demand, so on a fresh clone these files
#: died on FileNotFoundError before asserting anything. Both boards below
#: are in `git ls-files`.
QFN = _stage('routed_output.kicad_pcb', _STAGE)
TIGARD = _stage('tigard.kicad_pcb', _STAGE)

BAD = []


def want(cond, label, extra=''):
    if cond:
        print('  PASS: %s' % label)
    else:
        BAD.append(label)
        print('  FAIL: %s %s' % (label, extra))


def test_kicad_cli_returns_something_near_but_not_equal_to_the_asked_size():
    """The measured fact, pinned as a LIVE guard rather than left as a comment.

    Asking 640x480 came back 616x448 and asking 900x700 came back 872x672, so
    the panel must letterbox into a box it chose itself. If a future KiCad
    starts honouring the request exactly, this test says so loudly -- at which
    point the letterbox is still correct, just no longer load-bearing.
    """
    d = tempfile.mkdtemp()
    png, err = kir.render_iso(LVDS, os.path.join(d, 'a.png'), CLI, 640, 480)
    want(not err and png, 'a plain render succeeds', err)
    w, h = Image.open(png).size
    want(0.85 * 640 <= w <= 640 and 0.85 * 480 <= h <= 480,
         'the render comes back NEAR the requested size', '%dx%d' % (w, h))
    want((w, h) != (640, 480),
         'but NOT equal to it -- measured 616x448 for a 640x480 request, which '
         'is why the caller never trusts the returned size', '%dx%d' % (w, h))


def test_the_yaw_sweep_reaches_kicad_cli_and_changes_the_picture():
    """The plan's yaw is pinned; that it ARRIVES was not.

    Deleting `--rotate` from the argv entirely -- which would make every shot
    of every film identical and delete the feature's whole "animated for free"
    premise -- survived the suite. A pixel comparison is the only thing that
    can see it, because the plan is still perfectly correct when the flag is
    dropped.
    """
    from PIL import ImageChops
    d = tempfile.mkdtemp()
    a, ea = kir.render_iso(LVDS, os.path.join(d, 'y0.png'), CLI, 400, 300,
                           rotate=(-45.0, 0.0, 20.0))
    b, eb = kir.render_iso(LVDS, os.path.join(d, 'y1.png'), CLI, 400, 300,
                           rotate=(-45.0, 0.0, 80.0))
    want(not ea and not eb, 'both renders succeed', (ea, eb))
    ia = Image.open(a).convert('RGBA')
    ib = Image.open(b).convert('RGBA')
    want(ia.size == ib.size, 'same size', (ia.size, ib.size))
    want(ImageChops.difference(ia, ib).getbbox() is not None,
         'a 60-degree yaw change reaches kicad-cli and changes the pixels -- '
         'without this, dropping --rotate is invisible')
    c, _ = kir.render_iso(LVDS, os.path.join(d, 'y2.png'), CLI, 400, 300,
                          rotate=(-45.0, 0.0, 20.0))
    want(ImageChops.difference(ia, Image.open(c).convert('RGBA')).getbbox()
         is None,
         'while the SAME yaw renders the same pixels, so the difference above '
         'is the rotation and not noise')


def test_parallel_renders_are_deterministic_and_really_parallel():
    import threading
    d = tempfile.mkdtemp()
    boards = [LVDS, QFN, LVDS, QFN]

    def run(workers, tag):
        jobs = [(i, b, os.path.join(d, '%s_%d.png' % (tag, i)),
                 (-45.0, 0.0, 45.0 + 5 * i)) for i, b in enumerate(boards)]
        return kir.render_many(jobs, CLI, workers=workers, width=320, height=240)

    ser = run(1, 'ser')
    threads = set()
    real_map = kir.render_iso

    def counting(*a, **k):
        threads.add(threading.get_ident())
        return real_map(*a, **k)

    kir.render_iso = counting
    try:
        par = run(4, 'par')
    finally:
        kir.render_iso = real_map

    want(set(ser) == set(par), 'the same keys come back', (set(ser), set(par)))

    # PIXELS, not bytes. Measured: kicad-cli's PNG bytes are not stable even
    # against ITSELF -- two identical serial runs of the same job produced
    # pixel-identical images with different file bytes (key 3 of 4), so it
    # embeds something per-run in the stream. Comparing bytes here would have
    # been a flaky test dressed up as a determinism guarantee, and it would
    # have failed for a reason that has nothing to do with threading. This is
    # the repo's standing rule about hashing outputs, in a new artifact.
    from PIL import ImageChops
    diff = []
    for k in ser:
        a = Image.open(ser[k][0]).convert('RGBA')
        b = Image.open(par[k][0]).convert('RGBA')
        if a.size != b.size or ImageChops.difference(a, b).getbbox() is not None:
            diff.append(k)
    want(not diff,
         'and every render is PIXEL-identical at 1 worker and at 4, so the '
         'movie does not depend on how many cores you have', diff)
    want(len(threads) > 1,
         'and more than one thread really ran -- otherwise the determinism '
         'above would be satisfied by silently serialising', len(threads))


def test_reproducibility_is_the_default_qualitys_not_kicad_clis():
    """Which half of the determinism claim belongs to whom.

    Measured, two SERIAL renders of the same job:
      --quality basic : bytes and pixels identical, on both boards tried
      --quality high  : bytes and pixels BOTH differ, on both

    So the movie's reproducibility is a property of the default quality, not of
    kicad-cli, and the worker count is not what changes it. Worth a test because an
    earlier version of this file's comment had it backwards -- it claimed byte
    instability at `basic` on the strength of one noisy sample, and changed the
    determinism assertion from bytes to pixels to work around a problem that was
    not there.
    """
    from PIL import ImageChops
    d = tempfile.mkdtemp()

    def twice(quality):
        outs = []
        for n in (1, 2):
            p = os.path.join(d, '%s_%d.png' % (quality, n))
            png, err = kir.render_iso(LVDS, p, CLI, 320, 240, quality=quality)
            if not png:
                return None, err
            outs.append(png)
        same_bytes = open(outs[0], 'rb').read() == open(outs[1], 'rb').read()
        a = Image.open(outs[0]).convert('RGBA')
        b = Image.open(outs[1]).convert('RGBA')
        return (same_bytes, ImageChops.difference(a, b).getbbox() is None), ''

    basic, err = twice('basic')
    want(basic is not None, 'the basic renders succeed', err)
    want(basic == (True, True),
         'at the DEFAULT quality a render is reproducible in both bytes and '
         'pixels, so the movie is too', basic)

    high, err2 = twice('high')
    if high is None:
        print('  NOTE: --quality high did not render here (%s); skipping the '
              'other half' % err2)
        want(True, 'the high-quality arm is unavailable on this machine')
        return
    want(high != (True, True),
         'while at --quality high kicad-cli is not reproducible against ITSELF '
         'run to run -- nothing here can make it so, and that is the caveat the '
         'render_many docstring carries', high)


def test_a_real_board_with_no_resolvable_models_still_renders_and_says_bare():
    """tigard: 84 model refs -- 81 ${KISYS3DMOD} + 3 ${KIPRJMOD}, 82 of them
    .wrl -- against a KiCad 10 tree that ships .step only."""
    d = tempfile.mkdtemp()
    png, err = kir.render_iso(TIGARD, os.path.join(d, 'tigard.png'), CLI,
                              320, 240)
    want(png and not err,
         'a bare board is NOT a failure -- it renders fine, it just has no '
         'component bodies', err)
    m = kir.resolve_models(TIGARD, kir.model_dirs(CLI, TIGARD))
    want(m and m['total'] == 84, '84 model references', m and m['total'])
    if m and m['found'] == 0:
        want('BARE BOARD' in kir.models_note(m),
             'and with none of them on disk the note says BARE BOARD',
             kir.models_note(m))
    else:
        # A machine with the .wrl libraries installed is a legitimate state and
        # must not fail the suite -- but say which state it was in, so a reader
        # is never left guessing why this assertion did not run.
        print('  NOTE: this machine resolves %d/%d tigard models, so the '
              'bare-board caption does not apply here'
              % (m['found'], m['total']))
        want(True, 'the models are present on this machine, so no bare caption')


TESTS_TO_RUN = [
    test_kicad_cli_returns_something_near_but_not_equal_to_the_asked_size,
    test_the_yaw_sweep_reaches_kicad_cli_and_changes_the_picture,
    test_parallel_renders_are_deterministic_and_really_parallel,
    test_reproducibility_is_the_default_qualitys_not_kicad_clis,
    test_a_real_board_with_no_resolvable_models_still_renders_and_says_bare,
]


def main():
    print('kicad-cli: %s' % CLI)
    # Isolated, like the other #887 files: an exception in one test used to
    # abort the file, leaving every later test neither run nor reported --
    # indistinguishable, in the output, from tests that were never written.
    # A NameError from a bad edit is exactly how that was found.
    for fn in TESTS_TO_RUN:
        print('--- %s' % fn.__name__)
        try:
            fn()
        except Exception as exc:                            # noqa: BLE001
            import traceback
            BAD.append('%s RAISED %s' % (fn.__name__, exc))
            traceback.print_exc()
    if BAD:
        print('\nFAILED: %d' % len(BAD))
        for b in BAD:
            print('  - %s' % b)
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
