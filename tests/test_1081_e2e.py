#!/usr/bin/env python3
"""The stage3d film, end to end, through both front ends (#1081).

`make_movie` and `make_film.build_film` (stage3d, the only film layout)
on the film_chain fixture (placement glides, a flip to the back, copper) with
a converge ledger beside the boards:

  * **one frame size, the frame the layout planned**, whichever board fills
    the box -- the invariant `save_movie` cannot take a violation of;
  * **the 3D board, the X-ray on request, and the X-ray when a tool is
    missing produce the SAME frame count** -- the 3D board replaces pictures,
    never frames -- and each run SAYS which board it drew and why;
  * **the benchmark band is drawn and said**, and the placement panels and
    the attempts band are not (they are folded into it), nor is an iso panel
    (the board box is the 3D view).

The 3D arm self-skips its own checks, naming why, on a machine that cannot
render (no Node / `npm ci` / Chromium); the X-ray arms always run, and the
missing-tool arm expects the reason for whichever tool is missing FIRST --
its fake Node when the others are present, else (no `npm ci`) playwright-core.
"""
import contextlib
import io
import json
import os
import sys
import tempfile

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import importlib.util                                           # noqa: E402
if importlib.util.find_spec('PIL') is None:
    print('SKIP: needs Pillow')
    sys.exit(77)

import film_chain_1081 as FC                                    # noqa: E402
from stage3d import render3d as R3                              # noqa: E402

_FAIL = []
CAN_3D, WHY_3D = R3.available()


def _missing_reason():
    """`(why, token)` for the 'missing' arm, read under ITS environment:
    the reason `render3d.available` gives, which names whichever tool it
    finds missing FIRST. It checks playwright-core and a Chromium before it
    spawns Node, so on a machine with no `npm ci` the reason names
    playwright-core, not the fake Node the arm points at. `token` is what
    the film's line must carry: the fake Node's name when the other two
    tools are present, else the reason itself."""
    ok, why = R3.available()
    others = all(fn()[0] for fn in (R3.resolve_playwright,
                                    R3.resolve_browser))
    return why, ('no-node' if others else why)


def _check(ok, msg):
    print('  %s %s' % ('ok  ' if ok else 'FAIL', msg))
    if not ok:
        _FAIL.append(msg)


def _ledger(d):
    t0 = 1.7e9
    rows = [{'iteration': 0, 'kind': 'placement', 'accepted': True, 't': t0,
             'score': {'blocking': 40}},
            {'iteration': 1, 'kind': 'completion', 'accepted': True,
             't': t0 + 900, 'score': {'blocking': 3, 'quality': {
                 'vias': 80, 'copper_mm': 1500, 'segments': 950}}},
            {'iteration': 2, 'kind': 'completion', 'accepted': True,
             't': t0 + 1800, 'score': {'blocking': 0, 'quality': {
                 'vias': 74, 'copper_mm': 1450, 'segments': 910}}}]
    p = os.path.join(d, 'ledger.jsonl')
    with open(p, 'w') as f:
        for r in rows:
            f.write(json.dumps(r) + '\n')
    return p


def _movie(boards, led, png, **kw):
    import make_movie as MM
    err = io.StringIO()
    with contextlib.redirect_stderr(err):
        out = MM.make_movie(boards, out=os.path.join(png, 'film.mp4'),
                            size=960, camera='auto',
                            attempts_ledger=led, png_dir=png, quiet=False,
                            **kw)
    from PIL import Image
    pngs = sorted(p for p in os.listdir(png) if p.endswith('.png'))
    sizes = set()
    for p in pngs:
        with Image.open(os.path.join(png, p)) as im:
            sizes.add(im.size)
    fill = _board_fill(os.path.join(png, pngs[-1])) if pngs else 0.0
    return out, err.getvalue(), len(pngs), sizes, fill


def _board_fill(path):
    """How much of the board box the drawn board spans on the last frame,
    along whichever axis binds (the larger of the width and height
    fractions): the extent of pixels that differ from the box's own ground.
    The film-wide camera fit drew esp_prog's board at about a third of it
    (run 35), because it had to cover the pile and a mid-flip board."""
    from PIL import Image
    with Image.open(path) as im:
        im = im.convert('RGB')
        w, h = im.size
        bw = int(w * 0.68)                  # the box: left of the column
        y0, y1 = int(h * 0.10), int(h * 0.75)
        ground = im.getpixel((4, (y0 + y1) // 2))
        hit = [(x, y) for x in range(0, bw, 3) for y in range(y0, y1, 3)
               if sum(abs(a - b) for a, b in zip(im.getpixel((x, y)),
                                                 ground)) > 45]
    if not hit:
        return 0.0
    xs, ys = [q[0] for q in hit], [q[1] for q in hit]
    return max((max(xs) - min(xs)) / float(bw),
               (max(ys) - min(ys)) / float(y1 - y0))


def test_make_movie_stage3d_three_ways():
    with FC.Chain() as c:
        led = _ledger(c.dir)
        runs = {}
        expect_missing = None
        arms = [('2d', {'board3d': '2d'}, {}),
                ('knob', {}, {'KICAD_MOVIE_BOARD3D': '2d'}),
                ('missing', {}, {'KICAD_STAGE3D_NODE':
                                 os.path.join(c.dir, 'no-node.exe')})]
        if CAN_3D:
            arms.insert(0, ('3d', {}, {}))
        else:
            print('    (3D arm skipped: %s)' % WHY_3D)
        for name, kw, env in arms:
            old = {k: os.environ.get(k) for k in env}
            os.environ.update(env)
            import env_knobs                 # it reads the env at import
            env_knobs.refresh()
            try:
                if name == 'missing':
                    expect_missing = _missing_reason()
                png = tempfile.mkdtemp(prefix='t1081e_', dir=c.dir)
                runs[name] = _movie(c.boards, led, png, **kw)
            finally:
                for k, v in old.items():
                    if v is None:
                        os.environ.pop(k, None)
                    else:
                        os.environ[k] = v
                env_knobs.refresh()
        for name, (out, err, n, sizes, _fill) in runs.items():
            _check(out and os.path.isfile(out), '%s: a film was written' % name)
            _check(len(sizes) == 1 and (960, 540) in sizes,
                   '%s: one frame size, the planned 960x540 (%s)'
                   % (name, sizes))
            _check('benchmark band: converge' in err,
                   '%s: the benchmark band is drawn and said' % name)
            _check('placement' not in err.lower().replace(
                'placement laps', '') or 'panel' not in err.lower(),
                   '%s: no placement panel' % name)
        counts = {k: v[2] for k, v in runs.items()}
        _check(len(set(counts.values())) == 1,
               'every arm has the same frame count (%s)' % counts)
        if '3d' in runs:
            _check('stage3d: 3D board' in runs['3d'][1],
                   '3d: says it drew the 3D board')
            _check(runs['3d'][4] >= 0.6,
                   '3d: the board spans %.0f %% of its box (the binding '
                   'axis) on the '
                   'last frame (>= 60 %%: the camera is fitted per state)'
                   % (100 * runs['3d'][4]))
        _check('stage3d: 2D X-ray in the board box -- the 2D X-ray was '
               'asked for' in runs['2d'][1], '2d: says it was asked for')
        _check('the 2D X-ray was asked for' in runs['knob'][1],
               '$KICAD_MOVIE_BOARD3D=2d reaches a film that passed no flag '
               '(the GUI recorder\'s and place_route_loop\'s only way)')
        from stage3d import film as _s3f
        _check(not _s3f._LIVE, 'the 3D state frames are removed once the '
               'film is written, not at exit (%s)' % _s3f._LIVE)
        miss = runs['missing'][1]
        why, token = expect_missing
        lines = [l for l in miss.splitlines()
                 if l.startswith('stage3d: 2D X-ray in the board box -- ')]
        _check(len(lines) == 1 and why in lines[0] and token in lines[0],
               'a missing tool: the X-ray, and the reason names the tool '
               'missing first -- %r (%r)' % (token, lines))


def test_make_film_stage3d():
    import make_film as MF
    with FC.Chain() as c:
        led = _ledger(c.dir)
        shots = MF.parse_positional(c.boards, [])
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            frames = MF.build_film(shots, size=960, fps=6.0, camera='auto',
                                   quiet=True,
                                   placement={'ledger': led,
                                              'board3d': '2d'})
        frames = list(frames or [])
        sizes = {f.size for f in frames}
        _check(frames and len(sizes) == 1,
               'build_film: one frame size (%s)' % sizes)
        e = err.getvalue()
        _check('benchmark band: converge' in e,
               'build_film: the benchmark band is drawn and said')
        _check('the 2D X-ray was asked for' in e,
               'build_film: board3d reaches build_boards')


TESTS = (test_make_movie_stage3d_three_ways, test_make_film_stage3d)


def main():
    for fn in TESTS:
        print('%s:' % fn.__name__)
        fn()
    if _FAIL:
        print('')
        print('%d FAILURE(S)' % len(_FAIL))
        for msg in _FAIL:
            print('  - %s' % msg)
        return 1
    print('')
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
