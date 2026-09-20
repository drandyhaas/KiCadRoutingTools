#!/usr/bin/env python3
"""#946 SECTION 4: per-format defaults for `--size` and `--fps` -- REFUSED, on
the measurement.

The issue's fourth suggestion is not one of the design note's thirteen ranked
items, and a PR that autocloses #946 while silently dropping it makes a claim
that is not true. The issue's text:

    `DEFAULT_SIZE = 1000`, `DEFAULT_FPS = 6.0` ... 6 fps is below the rate at
    which motion reads as continuous

The premise is true of continuous motion and false of this movie, and the
measurement is what shows it. **Raising `fps` adds no frames.** A routing movie's
frames are DISCRETE EVENTS -- one per copper event, or per chunk of a coarse
reveal -- not samples of a continuous motion, so `fps` only changes the per-frame
delay. Measured on the two-board `fanout_starting_point -> fanout_output1`
chain, identical 13 frames at every rate:

    gif  1000 px   6 fps   59550 B   3580 ms of film
    gif  1000 px  24 fps   59550 B   1990 ms of film      <- same bytes, shorter
    mp4  1000 px   6 fps  108935 B
    mp4  1000 px  24 fps   96467 B                        <- SMALLER at 24 fps

So the raise is nearly free in bytes -- and it buys nothing, because it does not
interpolate: it plays the same slideshow faster and ends sooner. The thing that
actually makes motion continuous is MORE FRAMES, which is #1022 (a rip retracts
and its replacement grows over four stages) and the camera's `tween`, not a
higher rate over the same thirteen pictures.

**So the deliverable here is the gate, and the gate said no** -- the same shape
#1002 closed on. `DEFAULT_SIZE` / `DEFAULT_FPS` are shared by the CLI, the
stress renderer and the GUI button, so raising them would change every
stress-run artifact at once in exchange for a shorter film.

What this file pins:

  * the defaults are **still** 1000 / 6.0, so a later raise has to come past
    this measurement rather than around it;
  * `fps` does not change the FRAME COUNT -- which is the whole argument, and
    the one thing a reader would have to take on trust otherwise;
  * the film's DURATION falls as `fps` rises, which is the cost the issue's
    suggestion does not mention.

It re-measures rather than quoting: a measured number needs a committed
measurement, and a number in a docstring is a claim.
"""
import os
import subprocess
import sys
import tempfile

RUN_ALL_TIMEOUT = 600

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, os.path.join(ROOT, 'py_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from PIL import Image
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

BOARDS = [os.path.join(ROOT, 'kicad_files', 'fanout_starting_point.kicad_pcb'),
          os.path.join(ROOT, 'kicad_files', 'fanout_output1.kicad_pcb')]

#: The shipping default, and the raise the issue asks about.
BASE_FPS = 6.0
RAISED_FPS = 24.0

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def _render(out, fps, size=300):
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_router', 'make_movie.py')] + BOARDS + [
        '-o', out, '--size', str(size), '--fps', str(fps), '--quiet']
    p = subprocess.run(argv, cwd=ROOT, capture_output=True, text=True)
    if not os.path.exists(out):
        return None, (p.stderr or p.stdout or 'no output')[-300:]
    return out, ''


def _gif_stats(path):
    tot = n = 0
    with Image.open(path) as im:
        try:
            while True:
                tot += im.info.get('duration', 0)
                n += 1
                im.seek(im.tell() + 1)
        except EOFError:
            pass
    return n, tot


def test_the_defaults_are_still_the_measured_ones():
    """A later raise has to come past the measurement, not around it."""
    _mark = len(_FAIL)
    import make_movie
    if (make_movie.DEFAULT_SIZE, make_movie.DEFAULT_FPS) != (1000, 6.0):
        fail('the defaults moved to %s/%s. #946 section 4 asked for this raise '
             'and the measurement REFUSED it: fps adds no frames, so the film '
             'gets shorter, not smoother. Re-run this file and either record a '
             'new measurement that supports the raise, or put them back.'
             % (make_movie.DEFAULT_SIZE, make_movie.DEFAULT_FPS))
    else:
        print('    DEFAULT_SIZE=%d  DEFAULT_FPS=%g, as measured'
              % (make_movie.DEFAULT_SIZE, make_movie.DEFAULT_FPS))
    if len(_FAIL) == _mark:
        print('  PASS: the refusal is recorded ON the numbers')


def test_fps_does_not_add_frames():
    """THE ARGUMENT. A routing movie's frames are discrete EVENTS, not samples
    of a continuous motion, so a higher rate does not interpolate anything."""
    _mark = len(_FAIL)
    d = tempfile.mkdtemp()
    slow = os.path.join(d, 'slow.gif')
    fast = os.path.join(d, 'fast.gif')
    a, err = _render(slow, BASE_FPS)
    b, err2 = _render(fast, RAISED_FPS)
    if not a or not b:
        fail('BROKEN TEST: no film rendered (%s / %s)' % (err, err2))
        return
    n_slow, ms_slow = _gif_stats(a)
    n_fast, ms_fast = _gif_stats(b)
    if n_slow != n_fast:
        fail('%g fps rendered %d frames and %g fps rendered %d -- the whole '
             'refusal rests on the count being the same'
             % (BASE_FPS, n_slow, RAISED_FPS, n_fast))
    if n_slow < 3:
        fail('BROKEN FIXTURE: %d frames is too few to measure anything'
             % n_slow)
        return
    if ms_fast >= ms_slow:
        fail('%g fps did not shorten the film: %d ms vs %d ms'
             % (RAISED_FPS, ms_fast, ms_slow))
    else:
        print('    %d frames at both rates; the film goes %d ms -> %d ms'
              % (n_slow, ms_slow, ms_fast))
    if os.path.getsize(b) > os.path.getsize(a) * 1.05:
        fail('the raised rate cost %d bytes against %d -- the "nearly free" '
             'half of the measurement does not hold'
             % (os.path.getsize(b), os.path.getsize(a)))
    if len(_FAIL) == _mark:
        print('  PASS: a higher rate plays the same pictures faster, and that '
              'is all it does')


def test_the_thing_that_does_add_frames_is_motion():
    """The honest alternative, named: #1022 adds frames, `--fps` does not."""
    _mark = len(_FAIL)
    import animate_route as A
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(BOARDS[1])
    layers = list(pcb.board_info.copper_layers)
    rows_s, rows_v = A._board_rows(pcb, layers)
    r, ls = A._renderer(BOARDS[1], None, 200, 1, 150)
    counts = {}
    for hold in (0, 2):
        m = A.Movie(r, ls, rip_hold=hold)
        m.reveal_delta(rows_s, rows_v, 'routed', chunks=1)
        m.reveal_delta(rows_s[:-80], rows_v, 'reroute', chunks=1)
        m.add(rows_s[-80:], [], 'reroute', 'reroute')
        counts[hold] = len(m.frames)
    if counts[2] <= counts[0]:
        fail('motion added no frames: %s' % counts)
    else:
        print('    same beat: %d frames cut, %d frames with motion'
              % (counts[0], counts[2]))
    if len(_FAIL) == _mark:
        print('  PASS: frames are what buy continuity, and #1022 adds them')


TESTS = (
    test_the_defaults_are_still_the_measured_ones,
    test_fps_does_not_add_frames,
    test_the_thing_that_does_add_frames_is_motion,
)


def main():
    for fn in TESTS:
        print('%s:' % fn.__name__)
        fn()
    if _FAIL:
        print('')
        print('%d FAILURE(S)' % len(_FAIL))
        for m in _FAIL:
            print('  - %s' % m)
        return 1
    print('')
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
