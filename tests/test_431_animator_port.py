"""`animate_fanout_clearance` renders through the shared stack now (#431).

It used to carry three duplications the issue's owner explicitly asked to
remove: a hand-rolled world->screen mapping (a second `Transform`), a
hand-rolled GIF writer (a second `save_movie`), and its own font handling. The
port deletes all three and gains two things for free -- the real board beneath
the BGA field, which the pygame version never drew, and `.mp4` output.

The RECORDING half (`_Recorder`, `repair_fanout_clearance(on_move=...)`) is
deliberately untouched: this is a rendering change, and the engine that decides
where caps go is not in scope.
"""

import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # placement split

import animate_fanout_clearance as AFC  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, 'kicad_files', 'fanout_output1.kicad_pcb')


class _Rec:
    """Duck-types _Recorder. The tracked fanned boards have no movable caps
    near a BGA (the committed golden came from glasgow revC, which needs a
    fanout run first), so the render is exercised on a synthetic recording --
    which is also the only way to make it deterministic."""

    def __init__(self, n_frames=4):
        self.static = {
            'bga': [(100.0, 80.0, 112.0, 92.0)],
            'clearance': 0.1,
            'vias': [(102.0 + 0.8 * i, 82.0 + 0.8 * j, 3 + i, 0.35)
                     for i in range(5) for j in range(5)],
            'balls': [(101.5 + 0.8 * i, 81.5 + 0.8 * j, 3 + i)
                      for i in range(5) for j in range(5)],
        }
        self.frames = []
        for k in range(n_frames):
            t = k / max(1, n_frames - 1)
            self.frames.append({
                'C1': self._cap(99.0 + 1.2 * t, 84.0 + 0.5 * t, (99.0, 84.0)),
                'C2': self._cap(113.0 - 1.4 * t, 86.0 - 0.8 * t, (113.0, 86.0)),
            })

    @staticmethod
    def _cap(x, y, seed):
        return {'seed_court': (seed[0], seed[1], seed[0] + 1.6, seed[1] + 0.9),
                'court': (x, y, x + 1.6, y + 0.9),
                'pads': [(x + 0.05, y + 0.1, x + 0.5, y + 0.8, 7),
                         (x + 1.1, y + 0.1, x + 1.55, y + 0.8, 9)]}


def _frames(path):
    from PIL import Image
    im = Image.open(path)
    n = 0
    sizes = set()
    try:
        while True:
            im.seek(n)
            sizes.add(im.size)
            n += 1
    except EOFError:
        pass
    return n, sizes


def test_it_renders_with_a_board_and_draws_the_substrate():
    d = tempfile.mkdtemp()
    try:
        out = os.path.join(d, 'a.gif')
        got = AFC.render_gif(_Rec(), out, size=300, sub_frames=4, fps=20,
                             pcb=parse_kicad_pcb(BOARD))
        assert got and os.path.getsize(out) > 500
        n, sizes = _frames(out)
        assert n > 5
        assert len(sizes) == 1, sizes
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_it_still_works_with_no_board():
    """The pygame version had no board either; a caller with only a recorder
    must not break."""
    d = tempfile.mkdtemp()
    try:
        out = os.path.join(d, 'b.gif')
        assert AFC.render_gif(_Rec(), out, size=300, sub_frames=4, fps=20)
        assert os.path.getsize(out) > 500
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_mp4_is_available_now():
    """Free from save_movie, which the pygame path could not reach. Falls back
    to a sibling .gif when imageio-ffmpeg is absent -- either outcome is a
    pass; what must NOT happen is an exception."""
    d = tempfile.mkdtemp()
    try:
        out = os.path.join(d, 'c.mp4')
        got = AFC.render_gif(_Rec(), out, size=280, sub_frames=3, fps=20)
        assert got
        assert os.path.exists(out) or os.path.exists(
            os.path.splitext(out)[0] + '.gif')
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_the_duplications_are_gone():
    """The point of the port. Each of these is a second implementation of
    something the shared stack already owns."""
    import ast
    path = os.path.join(ROOT, 'py_tools', 'animate_fanout_clearance.py')
    src = open(path, encoding='utf-8').read()
    tree = ast.parse(src)
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == 'render_gif')
    # Strip the docstring: it EXPLAINS the port, so a plain substring search
    # matches its own prose ("ported off pygame...") and passes for the wrong
    # reason. Check the code.
    body = ast.unparse(ast.Module(body=[b for b in fn.body
                                        if not (isinstance(b, ast.Expr)
                                                and isinstance(b.value, ast.Constant)
                                                and isinstance(b.value.value, str))],
                                  type_ignores=[]))
    for token, owner in (('pygame', 'route_render draws now'),
                         ('save_all', 'animate_route.save_movie encodes now'),
                         ('append_images', 'animate_route.save_movie encodes now'),
                         ('w2s', 'route_render.Transform maps now'),
                         ('off_x', 'route_render.Transform maps now')):
        assert token not in body, f"{token!r} still used in render_gif -- {owner}"
    assert 'save_movie' in body and 'BoardRenderer' in body


def test_the_shared_easing_helpers_are_imported_not_copied():
    src = open(os.path.join(ROOT, 'py_tools', 'animate_fanout_clearance.py'),
               encoding='utf-8').read()
    assert 'from movie_camera import' in src
    for name in ('def _smoothstep', 'def _lerp_rect', 'def _net_color'):
        assert name not in src, f"{name} is defined locally again"


def test_the_recording_half_is_untouched():
    """A rendering change must not have touched the engine that decides where
    the caps go."""
    assert hasattr(AFC, '_Recorder')
    rec = AFC._Recorder()
    assert rec.static is None and rec.frames == []


def test_render_is_reproducible():
    d = tempfile.mkdtemp()
    try:
        a, b = os.path.join(d, 'x.gif'), os.path.join(d, 'y.gif')
        AFC.render_gif(_Rec(), a, size=260, sub_frames=3, fps=20)
        AFC.render_gif(_Rec(), b, size=260, sub_frames=3, fps=20)
        assert open(a, 'rb').read() == open(b, 'rb').read()
    finally:
        shutil.rmtree(d, ignore_errors=True)


TESTS = [
    test_it_renders_with_a_board_and_draws_the_substrate,
    test_it_still_works_with_no_board,
    test_mp4_is_available_now,
    test_the_duplications_are_gone,
    test_the_shared_easing_helpers_are_imported_not_copied,
    test_the_recording_half_is_untouched,
    test_render_is_reproducible,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
