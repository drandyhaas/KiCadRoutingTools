#!/usr/bin/env python3
"""The routing movie streams its frames, and has a frame budget (#1036).

Run 32 (glasgow_revC) rendered a 22-board chain whose per-segment route traces
made ~6100 frames; `make_movie` held every one as a Pillow image, ran four
post-passes over the list, and the process working set reached 29.5 GB before
it was stopped. These tests pin the fix:

  * `frame_spool.FrameSpool` behaves like the list it replaces -- append, index,
    assign, iterate, lazy per-frame transforms, sizes -- and never holds more
    than one decoded frame;
  * `make_movie` hands `save_movie` a SPOOL, and `save_movie` still takes the
    plain list `awx/evolve_movie.py` passes it;
  * memory stays bounded as the frame count grows, measured in a child
    process with the platform's own peak-RSS counter (no psutil needed):
    with imageio the .mp4 streams, so the same film at 6x the frames grows
    peak RSS by far less than the frames would occupy; without it the film
    falls back to a GIF, whose writer holds what it is given, so what is
    pinned there is the GIF's own bound -- at most `GIF_MAX_FRAMES` frames,
    so two films both over the cap cost the same;
  * a trace over the `--max-frames` budget falls back to the chunked reveal
    and SAYS so, and with no budget the same trace plays in full.

Needs Pillow; renders a small in-repo board, no kicad-cli.
"""
import json
import os
import subprocess
import sys
import tempfile

RUN_ALL_FAST_OK = False

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_tools'), os.path.join(ROOT, 'py_placer')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from PIL import Image
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import frame_spool   # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')

_FAIL = []
_SKIPPED = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


_TMPS = []


def _mkdtemp(prefix):
    """A temp dir `main()` removes when the run ends, pass or fail."""
    d = tempfile.mkdtemp(prefix=prefix)
    _TMPS.append(d)
    return d


def _write_trace(board, n_events, path):
    """A synthetic per-segment trace: one `route` event per segment of the
    board's own copper, so replaying it builds the board a segment at a time
    -- the shape a KICAD_ROUTE_TRACE=1 sidecar has."""
    import animate_route as a
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(board)
    layers = list(pcb.board_info.copper_layers)
    segs, _vias = a._board_rows(pcb, layers)
    ev = [{'event': 'route', 'net_name': 'n%d' % i, 'add_s': [row]}
          for i, row in enumerate(segs[:n_events])]
    with open(path, 'w') as f:
        json.dump({'layers': layers, 'events': ev}, f)
    return len(ev)


# --------------------------------------------------------------------------
def test_spool_is_a_list():
    _mark = len(_FAIL)
    with frame_spool.FrameSpool() as sp:
        for k in range(5):
            sp.append(Image.new('RGB', (40, 30), (k * 40, 0, 0)))
        if len(sp) != 5:
            fail('len is %d, expected 5' % len(sp))
        if sp[3].getpixel((0, 0)) != (120, 0, 0):
            fail('frames[3] read back %r' % (sp[3].getpixel((0, 0)),))
        if sp[-1].getpixel((0, 0)) != (160, 0, 0):
            fail('frames[-1] is not the last frame')
        sp[1] = Image.new('RGB', (40, 30), (0, 99, 0))
        if sp[1].getpixel((0, 0)) != (0, 99, 0):
            fail('an assigned frame did not read back')
        # a lazy transform grows every frame; nothing is decoded to register it
        frame_spool.transform(
            sp, lambda i, f: Image.new('RGB', (40, 50), f.getpixel((0, 0))),
            out_size=(40, 50))
        if frame_spool.frame_sizes(sp) != {(40, 50)}:
            fail('sizes after the transform: %r' % frame_spool.frame_sizes(sp))
        if [f.size for f in sp] != [(40, 50)] * 5:
            fail('iteration did not apply the transform')
        if sp[1].getpixel((0, 0)) != (0, 99, 0):
            fail('the transform lost the assigned frame')
        if sp.loaded > 1:
            fail('the spool held %d decoded frames' % sp.loaded)
        d = sp.dir
    if os.path.isdir(d):
        fail('close() left the spool directory behind: %s' % d)
    # a LIST goes through the same helper, in place, as every post-pass did
    lst = [Image.new('RGB', (10, 10)) for _ in range(3)]
    same = frame_spool.transform(lst, lambda i, f: f.resize((10, 20)))
    if same is not lst or {f.size for f in lst} != {(10, 20)}:
        fail('transform() on a list is not in place')
    if len(_FAIL) == _mark:
        print('  PASS: append/index/assign/iterate/transform/sizes, one '
              'decoded frame at a time, and a list still transforms in place')


def test_make_movie_hands_save_movie_a_spool_and_a_list_still_saves():
    _mark = len(_FAIL)
    import animate_route as a
    import make_movie
    seen = {}
    saved = a.save_movie

    def _spy(frames, out, *args, **kw):
        seen['type'] = type(frames).__name__
        seen['n'] = len(frames)
        return saved(frames, out, *args, **kw)
    a.save_movie = _spy
    tmp = _mkdtemp(prefix='t1036_')
    try:
        out = make_movie.make_movie([BOARD], out=os.path.join(tmp, 'm.gif'),
                                    size=300, quiet=True, attempts=False)
    finally:
        a.save_movie = saved
    if seen.get('type') != 'FrameSpool':
        fail('make_movie handed save_movie a %r, not a FrameSpool'
             % seen.get('type'))
    if not out or not os.path.isfile(out):
        fail('make_movie wrote nothing')
    else:
        print('    make_movie -> %s over a spool of %d frames'
              % (os.path.basename(out), seen.get('n', 0)))
    # the awx/evolve_movie.py call: a plain list
    lst = [Image.new('RGB', (64, 48), (i * 20, 0, 0)) for i in range(6)]
    g = os.path.join(tmp, 'list.gif')
    if not a.save_movie(lst, g, fps=6, end_hold=0.5) or not os.path.isfile(g):
        fail('save_movie no longer takes a plain list')
    if len(_FAIL) == _mark:
        print('  PASS: make_movie streams through a spool; save_movie still '
              'takes a list')


def test_gif_strides_over_its_cap():
    _mark = len(_FAIL)
    import animate_route as a
    tmp = _mkdtemp(prefix='t1036g_')
    n = a.GIF_MAX_FRAMES * 2 + 7
    with frame_spool.FrameSpool() as sp:
        for i in range(n):
            sp.append(Image.new('RGB', (32, 24), (i % 256, (i * 7) % 256, 0)))
        g = os.path.join(tmp, 's.gif')
        a.save_movie(sp, g, fps=6, end_hold=0.0)
    from PIL import ImageSequence
    with Image.open(g) as im:
        got = sum(1 for _ in ImageSequence.Iterator(im))
    if got > a.GIF_MAX_FRAMES + 2:
        fail('a %d-frame GIF kept %d frames, over the %d cap'
             % (n, got, a.GIF_MAX_FRAMES))
    if len(_FAIL) == _mark:
        print('  PASS: %d frames -> a %d-frame GIF (cap %d)'
              % (n, got, a.GIF_MAX_FRAMES))


def _peak_rss():
    """This process's PEAK resident set in bytes, from the platform itself:
    `GetProcessMemoryInfo` on Windows, `VmHWM` in /proc/self/status on Linux,
    `getrusage` elsewhere (bytes on macOS, KiB on other Unixes). None when
    none answers. No psutil: the suite image does not install it, and a
    memory check that skips wherever the suite runs pins nothing."""
    if sys.platform == 'win32':
        import ctypes
        from ctypes import wintypes

        class _PMC(ctypes.Structure):
            _fields_ = [('cb', wintypes.DWORD),
                        ('PageFaultCount', wintypes.DWORD),
                        ('PeakWorkingSetSize', ctypes.c_size_t),
                        ('WorkingSetSize', ctypes.c_size_t),
                        ('QuotaPeakPagedPoolUsage', ctypes.c_size_t),
                        ('QuotaPagedPoolUsage', ctypes.c_size_t),
                        ('QuotaPeakNonPagedPoolUsage', ctypes.c_size_t),
                        ('QuotaNonPagedPoolUsage', ctypes.c_size_t),
                        ('PagefileUsage', ctypes.c_size_t),
                        ('PeakPagefileUsage', ctypes.c_size_t)]
        try:
            k32 = ctypes.WinDLL('kernel32', use_last_error=True)
            k32.GetCurrentProcess.restype = wintypes.HANDLE
            fn = k32.K32GetProcessMemoryInfo
            fn.argtypes = [wintypes.HANDLE, ctypes.POINTER(_PMC),
                           wintypes.DWORD]
            fn.restype = wintypes.BOOL
            pmc = _PMC()
            pmc.cb = ctypes.sizeof(_PMC)
            if fn(k32.GetCurrentProcess(), ctypes.byref(pmc), pmc.cb):
                return int(pmc.PeakWorkingSetSize)
        except (OSError, AttributeError):
            pass
        return None
    try:
        with open('/proc/self/status', encoding='ascii') as f:
            for line in f:
                if line.startswith('VmHWM:'):
                    return int(line.split()[1]) * 1024
    except (OSError, ValueError, IndexError):
        pass
    try:
        import resource
        r = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        return int(r) if sys.platform == 'darwin' else int(r) * 1024
    except (ImportError, OSError, ValueError):
        return None


def _mp4_available():
    import importlib.util
    return all(importlib.util.find_spec(m) is not None
               for m in ('imageio', 'imageio_ffmpeg', 'numpy'))


def _child(n_events, out, mode='spool'):
    """Render the movie of a board built up by an `n_events` trace; print the
    peak RSS and the tracemalloc peak as JSON. `mode='list'` is the CONTROL:
    the same frames built into an in-memory list, as before #1036, so the
    measurement is shown able to see growth at all."""
    import tracemalloc
    import make_movie
    tmp = os.path.dirname(out)
    board = os.path.join(tmp, 'b.kicad_pcb')
    import shutil
    shutil.copy(BOARD, board)
    _write_trace(board, n_events, os.path.join(tmp, 'b_routetrace.json'))
    tracemalloc.start()
    if mode == 'list':
        import animate_route as a
        tr = os.path.join(tmp, 'b_routetrace.json')
        frames = a.build_boards([('route', board, tr)], board, 500, 1, 150,
                                2, 6, max_frames=0)
        got = len(frames)
    else:
        got = make_movie.make_movie([board], out=out, size=500, quiet=True,
                                    attempts=False, max_frames=0, fps=30)
    _cur, peak = tracemalloc.get_traced_memory()
    print(json.dumps({'peak_rss': _peak_rss(), 'py_peak': peak, 'out': got}))


def test_memory_is_bounded_in_the_frame_count():
    _mark = len(_FAIL)
    import animate_route as a
    if _peak_rss() is None:
        # NOT a pass: said as a skip, and counted, so the summary line cannot
        # read "all checks passed" over a measurement that never ran.
        _SKIPPED.append('memory: this platform reports no peak RSS')
        print('  SKIP: this platform reports no peak RSS')
        return
    if _mp4_available():
        # the .mp4 STREAMS: memory must not follow the frame count at all
        lo, hi, ext = 100, 600, '.mp4'
        what = 'mp4 (streamed)'
    else:
        # no imageio: the film falls back to a GIF, and Pillow's GIF writer
        # holds every frame it is handed -- which the stride caps at
        # GIF_MAX_FRAMES. So the pinned bound is the CAP: two films BOTH over
        # it must cost the same, however many frames each has.
        lo = a.GIF_MAX_FRAMES + 40
        hi, ext = 3 * lo, '.gif'
        what = 'GIF (no imageio; strided to %d frames)' % a.GIF_MAX_FRAMES
    print('    arm: %s, %d vs %d events' % (what, lo, hi))
    import shutil
    res = {}
    for key, n, mode in ((lo, lo, 'spool'), (hi, hi, 'spool'),
                         ('c_lo', lo, 'list'), ('c_hi', hi, 'list')):
        tmp = tempfile.mkdtemp(prefix='t1036m_')
        try:
            r = subprocess.run([sys.executable, '-X', 'utf8', __file__,
                                '--child', str(n), os.path.join(tmp, 'm' + ext),
                                mode],
                               capture_output=True, text=True, timeout=1200)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
        line = [ln for ln in r.stdout.splitlines() if ln.startswith('{')]
        if r.returncode != 0 or not line:
            fail('BROKEN: %s child for %d events exited %d: %s'
                 % (mode, n, r.returncode, (r.stderr or r.stdout)[-400:]))
            return
        res[key] = json.loads(line[-1])
        if not res[key].get('peak_rss'):
            fail('BROKEN: the %s child for %d events reported no peak RSS'
                 % (mode, n))
            return
    more = hi - lo
    # THE CONTROL: the same frames held in a list must be SEEN to grow, or a
    # flat reading below proves nothing about streaming.
    c_rss = res['c_hi']['peak_rss'] - res['c_lo']['peak_rss']
    print('    CONTROL (in-memory list): +%.1f MB peak RSS for %d more frames'
          % (c_rss / 1e6, more))
    # a 500 px frame of this board is ~500x310x3 = 0.46 MB; `more` frames
    # held in memory would cost that many times it. The film must grow by
    # far less.
    frame_bytes = 500 * 310 * 3
    held = more * frame_bytes
    d_rss = res[hi]['peak_rss'] - res[lo]['peak_rss']
    d_py = res[hi]['py_peak'] - res[lo]['py_peak']
    print('    peak RSS   %d events: %6.1f MB   %d events: %6.1f MB  '
          '(+%.1f MB; holding %d more frames would be +%.0f MB)'
          % (lo, res[lo]['peak_rss'] / 1e6, hi, res[hi]['peak_rss'] / 1e6,
             d_rss / 1e6, more, held / 1e6))
    print('    tracemalloc peak      %6.1f MB              %6.1f MB  (+%.1f MB)'
          % (res[lo]['py_peak'] / 1e6, res[hi]['py_peak'] / 1e6, d_py / 1e6))
    if c_rss < held * 0.5:
        fail('BROKEN: the in-memory control grew only %.1f MB, so this '
             'measurement cannot see held frames' % (c_rss / 1e6))
    if d_rss > held * 0.25:
        fail('%s: peak RSS grew %.1f MB for %d more frames -- over a quarter '
             'of what holding them would cost (%.0f MB)'
             % (what, d_rss / 1e6, more, held / 1e6))
    if d_py > 40e6:
        fail('the Python heap grew %.1f MB for %d more frames'
             % (d_py / 1e6, more))
    if len(_FAIL) == _mark:
        print('  PASS: %s -- %dx the frames, peak RSS +%.1f MB'
              % (what, hi // lo, d_rss / 1e6))


def test_a_trace_over_budget_falls_back_loudly():
    _mark = len(_FAIL)
    import animate_route as a
    tmp = _mkdtemp(prefix='t1036b_')
    import shutil
    board = os.path.join(tmp, 'b.kicad_pcb')
    shutil.copy(BOARD, board)
    tr = os.path.join(tmp, 'b_routetrace.json')
    n = _write_trace(board, 300, tr)
    steps = [('route', board, tr)]
    notes = []
    fr = a.build_boards(steps, board, 200, 1, 150, 2, 6, max_frames=50,
                        notes=notes)
    if not notes or 'chunks' not in notes[0]:
        fail('a %d-event trace under a 50-frame budget left no note: %r'
             % (n, notes))
    if len(fr) > 50:
        fail('the over-budget film has %d frames, over its 50-frame budget'
             % len(fr))
    full = a.build_boards(steps, board, 200, 1, 150, 2, 6, max_frames=0)
    if len(full) < n:
        fail('with no budget the trace played %d frames for %d events'
             % (len(full), n))
    est = a.trace_frame_estimate(a.load_trace(tr), 2)
    if est < n:
        fail('trace_frame_estimate %d is under the %d add events' % (est, n))
    if len(_FAIL) == _mark:
        print('  PASS: %d events -> %d frames under budget 50 (%s); %d frames '
              'with no budget' % (n, len(fr), notes[0][:60], len(full)))


def test_the_spool_says_when_the_disk_cannot_hold_it():
    """The spool trades RAM for DISK (~1.27 MB per 1400 px frame). When the
    temp dir cannot hold the estimate the movie says so LOUDLY, and an
    UNBUDGETED film falls back to the default budget."""
    _mark = len(_FAIL)
    import contextlib
    import io
    import make_movie
    saved = frame_spool.disk_check
    frame_spool.disk_check = lambda d, n, px: (False, 10 ** 12, 10 ** 9)
    err = io.StringIO()
    tmp = _mkdtemp(prefix='t1036d_')
    try:
        with contextlib.redirect_stderr(err):
            make_movie.make_movie([BOARD], out=os.path.join(tmp, 'm.gif'),
                                  size=200, quiet=True, attempts=False,
                                  max_frames=0)
    finally:
        frame_spool.disk_check = saved
    e = err.getvalue()
    if 'SPOOL DISK' not in e or 'falling back' not in e:
        fail('a spool the disk cannot hold was not reported: %r' % e[-300:])
    ok, need, free = frame_spool.disk_check(None, 6000, 1400 * 788)
    print('    6000 frames at 1400x788 would spool ~%.1f GB (free here: %s)'
          % (need / 1e9, '%.0f GB' % (free / 1e9) if free else 'unknown'))
    if not 6.5e9 < need < 9e9:
        fail('the 6000-frame estimate is %.1f GB, not ~7.6' % (need / 1e9))
    if len(_FAIL) == _mark:
        print('  PASS: reported, and the unbudgeted film fell back to the '
              'budget')


def test_make_film_streams_through_a_spool():
    """make_film's CLI hands save_movie a FrameSpool too (#1036 verifier: it
    still collected every frame in a list)."""
    _mark = len(_FAIL)
    import animate_route as a
    import make_film
    seen = {}
    saved = a.save_movie

    def _spy(frames, out, *args, **kw):
        seen['type'] = type(frames).__name__
        seen['n'] = len(frames)
        return saved(frames, out, *args, **kw)
    a.save_movie = _spy
    tmp = _mkdtemp(prefix='t1036f_')
    try:
        rc = make_film.main([BOARD, BOARD, '-o', os.path.join(tmp, 'f.gif'),
                             '--camera', 'off', '--quiet', '--size', '200',
                             '--no-attempts'])
    finally:
        a.save_movie = saved
    if rc != 0 or seen.get('type') != 'FrameSpool':
        fail('make_film handed save_movie a %r (rc %r)' % (seen.get('type'),
                                                          rc))
    if len(_FAIL) == _mark:
        print('  PASS: make_film streams %d frames through a spool'
              % seen.get('n', 0))


TESTS = (
    test_spool_is_a_list,
    test_make_movie_hands_save_movie_a_spool_and_a_list_still_saves,
    test_gif_strides_over_its_cap,
    test_a_trace_over_budget_falls_back_loudly,
    test_memory_is_bounded_in_the_frame_count,
    test_the_spool_says_when_the_disk_cannot_hold_it,
    test_make_film_streams_through_a_spool,
)


def main():
    if len(sys.argv) >= 4 and sys.argv[1] == '--child':
        _child(int(sys.argv[2]), sys.argv[3],
               sys.argv[4] if len(sys.argv) > 4 else 'spool')
        return 0
    try:
        for fn in TESTS:
            print('%s:' % fn.__name__)
            fn()
    finally:
        import shutil
        for d in _TMPS:
            shutil.rmtree(d, ignore_errors=True)
    if _FAIL:
        print('')
        print('%d FAILURE(S)' % len(_FAIL))
        for m in _FAIL:
            print('  - %s' % m)
        return 1
    print('')
    if _SKIPPED:
        print('%d of %d checks ran and passed; SKIPPED: %s'
              % (len(TESTS) - len(_SKIPPED), len(TESTS), '; '.join(_SKIPPED)))
        return 0
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
