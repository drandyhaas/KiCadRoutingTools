#!/usr/bin/env python3
"""The Blender hi-fi backend for the stage3d board (#1089).

Self-skips (exit 77) without a Blender, naming why. With one, on the
film_chain fixture's own scene and timeline:

  * **it renders every state** of the SAME timeline the three.js backend
    renders (a flip, a glide, copper), through `render3d.render(backend=
    'blender')`, and says it ran Cycles on the CPU;
  * **twice, byte-identical** -- a fixed seed, fixed samples, no denoiser,
    no stamp, the CPU device: a render that depends on the machine's GPU
    could not be pinned;
  * **the pictures carry the story**: the flipped state is not the front,
    and the copper changes the picture;
  * **the film takes it**: `stage3d.film.apply(mode='blender')` maps every
    frame onto it, and says so.
"""
import hashlib
import os
import shutil
import sys
import tempfile

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import importlib.util                                           # noqa: E402
if importlib.util.find_spec('PIL') is None:
    print('SKIP: needs Pillow')
    sys.exit(77)

from stage3d import render3d as R3                              # noqa: E402

BLENDER, WHY = R3.resolve_blender()
if not BLENDER:
    print('SKIP: the Blender backend cannot run on this machine: %s' % WHY)
    sys.exit(77)

from PIL import Image, ImageChops                               # noqa: E402
import film_chain_1081 as FC                                    # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
from stage3d import scene as SC                                 # noqa: E402
from stage3d import timeline as TL                              # noqa: E402

_FAIL = []
os.environ.setdefault('KICAD_STAGE3D_BLENDER_SAMPLES', '4')


def _check(ok, msg):
    print('  %s %s' % ('ok  ' if ok else 'FAIL', msg))
    if not ok:
        _FAIL.append(msg)


def _sha(p):
    with open(p, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def _diff(a, b):
    with Image.open(a) as x, Image.open(b) as y:
        d = ImageChops.difference(x.convert('RGB'), y.convert('RGB'))
        return d.convert('L').point(
            lambda q: 255 if q > 8 else 0).histogram()[255]


def _few_states(tl, keep):
    """The timeline cut to a few states -- Cycles is seconds per state."""
    out = dict(tl)
    out['states'] = [tl['states'][i] for i in keep]
    out['frames'] = list(range(len(keep)))
    return out


def test_blender_renders_the_same_timeline_deterministically():
    tmp = tempfile.mkdtemp(prefix='t1089_')
    try:
        with FC.Chain() as c:
            out = {}
            tr = FC.rip_trace(c.boards[-1], os.path.join(c.dir, 'tr.json'))
            _f, _m, st, _g = FC.film(c.boards, stage_out=out, traces={3: tr})
            tl = TL.build(out)
            fr = tl['frames']
            a0, b0 = [(x, y) for k, x, y in st.frame_log() if k == 'flip'][0]
            keep = sorted({fr[a0 - 1], fr[b0 + 1], fr[-1]})
            small = _few_states(tl, keep)
            sc = SC.build_scene(parse_kicad_pcb(c.boards[-1]))
            W, H = 240, 160
            a, ia, wa = R3.render(sc, small, width=W, height=H,
                                  out_dir=os.path.join(tmp, 'a'),
                                  backend='blender')
            b, ib, wb = R3.render(sc, small, width=W, height=H,
                                  out_dir=os.path.join(tmp, 'b'),
                                  backend='blender')
            print('    %s' % wa)
            _check(a is not None and b is not None,
                   'both renders produced frames (%s / %s)' % (wa, wb))
            if a is None or b is None:
                return
            _check('Cycles CPU' in str(ia.get('renderer')),
                   'it ran Cycles on the CPU (%r)' % ia.get('renderer'))
            same = sum(1 for x, y in zip(a, b) if _sha(x) == _sha(y))
            _check(same == len(a), 'two renders are byte-identical (%d of %d)'
                   % (same, len(a)))
            with Image.open(a[0]) as im:
                _check(im.size == (W, H), 'frames are the box size (%s)'
                       % (im.size,))
            idx = {s: i for i, s in enumerate(keep)}
            front, back, last = (idx[fr[a0 - 1]], idx[fr[b0 + 1]],
                                 idx[fr[-1]])
            _check(_diff(a[front], a[back]) > 300,
                   'the back is not the front')
            _check(back == last or _diff(a[back], a[last]) > 20,
                   'the copper the film reveals changes the picture')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_the_film_takes_the_blender_board():
    """`--board-3d blender`: build_boards maps every frame onto Cycles'
    pictures, all or nothing, and says which backend ran."""
    import contextlib
    import io as _io
    import animate_route as A
    import movie_camera as MC
    with FC.Chain() as c:
        err = _io.StringIO()
        st = MC.Stage(MC.synth_rounds(c.boards), '', tween=4, quiet=True)
        steps = [('step %d' % i, b, None) for i, b in enumerate(c.boards)]
        with contextlib.redirect_stderr(err):
            frames = A.build_boards(steps, c.boards[-1], 400, 1, None, 2, 6,
                                    stage=st, board3d='blender')
        frames = list(frames)
        e = err.getvalue()
        _check('stage3d: 3D board' in e and 'Cycles CPU' in e,
               'the film says it drew the Blender board (%s)'
               % [ln for ln in e.splitlines() if 'stage3d' in ln])
        _check(frames and len({f.size for f in frames}) == 1,
               'one frame size across %d frames' % len(frames))


TESTS = (test_blender_renders_the_same_timeline_deterministically,
         test_the_film_takes_the_blender_board)


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
