"""`route_render` gains crop/zoom/pan, an overlay hook and per-pad fills (#431).

All four seams exist so a PLACEMENT renderer can reuse this copper renderer
instead of copying it -- the issue's binding instruction is "reuse as much of
the existing image/movie creation code as possible". This file's job is the
other half of that bargain: proving the seams changed nothing for the four
existing callers (`route.py --preview-png`, `animate_route`, `render_run`,
`render_board_file`).

Deliberately NOT a committed golden-image hash. Pillow's LANCZOS resampling and
default-font rendering drift between versions, so a stored checksum would
red-fail on a contributor's box for reasons that have nothing to do with this
code. Every assertion below is either exact arithmetic or an in-environment
comparison of two images produced by the same Pillow.
"""

import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# #522 moved the engine modules out of the repo root into py_router/ (and the
# placement family into py_placer/, the leaf tools into py_tools/).
for _p in (_ROOT, os.path.join(_ROOT, 'py_router'),
           os.path.join(_ROOT, 'py_placer'), os.path.join(_ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from PIL import Image, ImageChops, ImageDraw  # noqa: E402

from kicad_parser import parse_kicad_pcb  # noqa: E402
from route_render import (BoardRenderer, Transform, _geometry_bounds,  # noqa: E402
                          load_font)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, 'kicad_files', 'interf_u_unrouted.kicad_pcb')


def _pcb():
    return parse_kicad_pcb(BOARD)


def test_default_view_transform_is_arithmetically_unchanged():
    """`view=None` must reproduce the pre-change transform exactly. Recomputed
    from the formula rather than compared to a stored number, so this stays a
    check on the CODE and not on a snapshot of it."""
    pcb = _pcb()
    r = BoardRenderer(pcb, size=800, supersample=2)
    bounds = pcb.board_info.board_bounds or _geometry_bounds(pcb)
    want = Transform(bounds, r.W * r.ss, r.H * r.ss, 0.03 * 800 * r.ss)
    assert abs(r.tf.scale - want.scale) < 1e-12, (r.tf.scale, want.scale)
    assert abs(r.tf.off_x - want.off_x) < 1e-12
    assert abs(r.tf.off_y - want.off_y) < 1e-12
    assert (r.tf.min_x, r.tf.min_y) == (want.min_x, want.min_y)


def test_frame_and_render_unchanged_by_the_new_kwargs():
    pcb = _pcb()
    r = BoardRenderer(pcb, size=600, supersample=1)
    assert r.frame().tobytes() == r.frame(overlays=None).tobytes()
    assert r.frame().tobytes() == r.frame(overlays=()).tobytes()
    assert (BoardRenderer(pcb, size=600, supersample=1).render().tobytes()
            == BoardRenderer(pcb, size=600, supersample=1, view=None).render().tobytes())


def test_public_draw_pads_paints_exactly_what_the_base_did():
    """The pad refactor threads a `fill` through four shape branches (custom
    polygons, circle, capsule, rounded rect) plus the drill. This proves
    `fill_for=None` still paints the original colour through every one of them,
    which a smoke test that merely calls draw_pads would not."""
    pcb = _pcb()
    with_pads = BoardRenderer(pcb, size=600, supersample=1)._base
    manual = BoardRenderer(pcb, size=600, supersample=1, show_pads=False)._base.copy()
    BoardRenderer(pcb, size=600, supersample=1).draw_pads(ImageDraw.Draw(manual))
    assert ImageChops.difference(with_pads, manual).getbbox() is None, \
        "draw_pads(fill_for=None) no longer reproduces the original pad fill"


def test_fill_for_actually_overrides():
    pcb = _pcb()
    r = BoardRenderer(pcb, size=600, supersample=1, show_pads=False)
    a, b = r._base.copy(), r._base.copy()
    r.draw_pads(ImageDraw.Draw(a))
    r.draw_pads(ImageDraw.Draw(b), fill_for=lambda p: (255, 0, 255))
    assert ImageChops.difference(a, b).getbbox() is not None


def test_view_is_a_real_crop_and_keeps_the_frame_size():
    """A viewport IS a pan and a zoom. Frame SIZE must not follow it, or a
    camera move mid-movie changes frame dimensions and `_write_mp4` silently
    degrades the whole thing to GIF."""
    pcb = _pcb()
    full = BoardRenderer(pcb, size=600, supersample=1)
    min_x, min_y, max_x, max_y = full.bounds
    # A real zoom shrinks BOTH dimensions. Cropping only one does not magnify --
    # Transform min()-fits, so the untouched dimension stays limiting. That is
    # letterboxing behaving correctly, and it is what test_odd_aspect covers.
    w, h = max_x - min_x, max_y - min_y
    quarter = (min_x + w / 4, min_y + h / 4, max_x - w / 4, max_y - h / 4)
    crop = BoardRenderer(pcb, size=600, supersample=1, view=quarter)

    assert (crop.W, crop.H) == (full.W, full.H), "view changed the frame size"
    assert crop.frame().size == full.frame().size
    assert crop.tf.scale > full.tf.scale * 1.5, "cropping to a quarter must magnify"
    # the board's far corner is now off-canvas, and the view centre lands mid-canvas
    assert crop.tf.pt(max_x, max_y)[0] > crop.W * crop.ss
    cx, cy = crop.tf.pt((quarter[0] + quarter[2]) / 2, (quarter[1] + quarter[3]) / 2)
    assert abs(cx - crop.W * crop.ss / 2) < 2 and abs(cy - crop.H * crop.ss / 2) < 2


def test_set_view_round_trips():
    pcb = _pcb()
    r = BoardRenderer(pcb, size=600, supersample=1)
    before = r.frame().tobytes()
    min_x, min_y, max_x, max_y = r.bounds
    r.set_view((min_x, min_y, min_x + (max_x - min_x) / 3, max_y))
    assert r.frame().tobytes() != before
    r.set_view(None)
    assert r.frame().tobytes() == before, "set_view(None) must restore the board view"


def test_odd_aspect_view_letterboxes_instead_of_distorting():
    """Transform min()-fits and centres, so a view whose aspect differs from the
    canvas letterboxes. That is what makes constant frame size safe."""
    pcb = _pcb()
    r = BoardRenderer(pcb, size=600, supersample=1)
    min_x, min_y, max_x, max_y = r.bounds
    tall = (min_x, min_y, min_x + (max_x - min_x) * 0.15, max_y)
    r.set_view(tall)
    # uniform scale => a square in world space is still square in pixels
    p0, p1 = r.tf.pt(min_x, min_y), r.tf.pt(min_x + 1.0, min_y + 1.0)
    assert abs((p1[0] - p0[0]) - (p1[1] - p0[1])) < 1e-9


def test_overlay_draws_and_is_registered_to_the_transform():
    """An overlay must land where the world transform says, not merely somewhere
    -- that is the difference between a hooked overlay and a decorative one."""
    pcb = _pcb()
    r = BoardRenderer(pcb, size=600, supersample=1)
    min_x, min_y, max_x, max_y = r.bounds
    wx, wy = (min_x + max_x) / 2, (min_y + max_y) / 2

    def mark(d, rr):
        px, py = rr.tf.pt(wx, wy)
        rad = rr.tf.length(2.0)
        d.ellipse([px - rad, py - rad, px + rad, py + rad], fill=(255, 0, 255))

    plain, marked = r.frame(), r.frame(overlays=[mark])
    box = ImageChops.difference(plain, marked).getbbox()
    assert box is not None, "overlay drew nothing"
    ex, ey = r.tf.pt(wx, wy)
    assert box[0] <= ex <= box[2] and box[1] <= ey <= box[3], \
        f"overlay landed at {box}, not around the transformed point ({ex:.1f},{ey:.1f})"


def test_overlays_run_in_order():
    pcb = _pcb()
    r = BoardRenderer(pcb, size=400, supersample=1)
    seen = []
    r.frame(overlays=[lambda d, rr: seen.append('a'), lambda d, rr: seen.append('b')])
    assert seen == ['a', 'b']


def test_load_font_is_memoized_and_survives_old_pillow():
    assert load_font(14) is load_font(14)
    assert load_font(6) is not None and load_font(1) is not None   # clamped, no raise


def test_board_with_no_edge_cuts_still_renders():
    """`_geometry_bounds` is the documented fallback; set_view must not assume
    board_bounds exists (the renderer is the one tool that must work on a board
    the optimizer refuses)."""
    pcb = _pcb()
    pcb.board_info.board_bounds = None
    r = BoardRenderer(pcb, size=400, supersample=1)
    assert r.frame().size == (r.W, r.H)
    r.set_view(None)
    assert r.bounds == _geometry_bounds(pcb)


TESTS = [
    test_default_view_transform_is_arithmetically_unchanged,
    test_frame_and_render_unchanged_by_the_new_kwargs,
    test_public_draw_pads_paints_exactly_what_the_base_did,
    test_fill_for_actually_overrides,
    test_view_is_a_real_crop_and_keeps_the_frame_size,
    test_set_view_round_trips,
    test_odd_aspect_view_letterboxes_instead_of_distorting,
    test_overlay_draws_and_is_registered_to_the_transform,
    test_overlays_run_in_order,
    test_load_font_is_memoized_and_survives_old_pillow,
    test_board_with_no_edge_cuts_still_renders,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
