#!/usr/bin/env python3
"""#887: the half that needs a REAL kicad-cli. Self-skips without one.

`tests/test_887_two_panel_frame.py` grades the composition with kicad-cli
monkeypatched and carries the regression on any machine. This file grades the
three things only a real render can answer, and it exists because each of them
was measured once and would otherwise be a comment nobody re-checks:

  * kicad-cli does NOT return the size you asked for;
  * parallel renders are byte-identical to serial ones, and really do run in
    parallel;
  * a board whose 3D models do not resolve still renders, and SAYS it is bare.
"""
import os
import subprocess
import sys
import tempfile

RUN_ALL_TIMEOUT = 900

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

try:
    from PIL import Image, ImageSequence
except ImportError:
    print('SKIP: Pillow is not installed')
    sys.exit(77)

import animate_route as A                                  # noqa: E402
import make_movie as MM                                    # noqa: E402
import movie_panels as mp                                  # noqa: E402
import kicad_iso_render as kir                             # noqa: E402

CLI, WHY = kir.resolve_cli()
if not CLI:
    print('SKIP: no kicad-cli on this machine (%s). '
          'tests/test_887_two_panel_frame.py carries the regression.' % WHY)
    sys.exit(77)

LVDS = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
QFN = os.path.join(ROOT, 'kicad_files', 'qfn_fanned_out.kicad_pcb')
TIGARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')

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


def test_the_two_panel_movie_writes_and_the_encoded_file_holds_one_size():
    """Assert on the ENCODED file, the way test_431_animator_port.py:99 does."""
    d = tempfile.mkdtemp()
    out = os.path.join(d, 'm.gif')
    got = MM.make_movie([LVDS, QFN], out=out, size=240, quiet=True,
                        panels='xray+iso',
                        iso_opts=mp.IsoOpts(max_renders=2, quality='basic'))
    want(got and os.path.exists(got), 'the two-panel movie is written', got)
    with Image.open(got) as im:
        sizes = {f.size for f in ImageSequence.Iterator(im)}
    want(len(sizes) == 1, 'and every encoded frame is one size', sizes)
    w, h = sizes.pop()
    want(h % 2 == 0, 'with an even height', h)


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


def test_a_real_board_with_no_resolvable_models_still_renders_and_says_bare():
    """tigard: 84 model refs, all .wrl, against a KiCad 10 tree that ships .step."""
    d = tempfile.mkdtemp()
    steps, final = MM.resolve_inputs([TIGARD])
    marks = []
    frames = A.build_boards(steps, final, 200, 1, 150, 2, 6, marks=marks)
    out, rep = mp.compose_two_panel(frames, marks, final,
                                    mp.IsoOpts(max_renders=1))
    want(rep['state'] == 'ran',
         'a bare board is NOT a failure -- it renders fine, it just has no '
         'component bodies', rep['state'])
    want(len({f.size for f in out}) == 1, 'one frame size')
    m = rep['models']
    want(m and m['total'] == 84, '84 model references', m and m['total'])
    if m and m['found'] == 0:
        want('BARE BOARD' in mp.iso_status_line(rep),
             'and with none of them on disk the status line says BARE BOARD',
             mp.iso_status_line(rep))
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
    test_the_two_panel_movie_writes_and_the_encoded_file_holds_one_size,
    test_parallel_renders_are_deterministic_and_really_parallel,
    test_a_real_board_with_no_resolvable_models_still_renders_and_says_bare,
]


def main():
    print('kicad-cli: %s' % CLI)
    for fn in TESTS_TO_RUN:
        print('--- %s' % fn.__name__)
        fn()
    if BAD:
        print('\nFAILED: %d' % len(BAD))
        for b in BAD:
            print('  - %s' % b)
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
