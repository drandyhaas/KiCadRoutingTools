#!/usr/bin/env python3
"""The design system's last visible defects, from PR C's r2 stills (#946, #1036).

  1. **The board fills its box.** Every routing frame after a glide was drawn
     at the camera's pile-inclusive overview, so the board sat ~600x370 in a
     980x594 box. The camera now settles on the board before copper steps and
     ends on it; a static frame and the end of a camera film both fill >=85%
     of the box on the limiting axis.
  2. **The rail's left title is never a later board's name.**
  3. (The iso caption's check went with the iso panel: stage3d is the only
     film layout.)
  4. **Layer cells are shaped like the board**, in a 2x2 grid in a tall
     column, one row in a wide box.
  5. **Every panel keeps the gutter**: no text within `gutter_px` of its box.
  6. **The event key lives in the rail** on a layout that has one, never over
     the board.

Needs Pillow; renders small in-repo boards, no kicad-cli.
"""
import os
import sys

# stage3d is the only film layout, so an unnamed layout is a stage3d
# frame. These tests grade the 2D board, not the Node/Chromium 3D
# render: set before env_knobs is read.
os.environ.setdefault('KICAD_MOVIE_BOARD3D', '2d')

RUN_ALL_TIMEOUT = 900

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_placer'), os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from PIL import Image, ImageDraw
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import animate_route as A          # noqa: E402
import frame_layout as FL          # noqa: E402
import render_chrome as RC         # noqa: E402
import render_panels as RP         # noqa: E402
import render_theme as RT          # noqa: E402

KF = os.path.join(ROOT, 'kicad_files')
ROUTED = os.path.join(KF, 'routed_output.kicad_pcb')
SEED = os.path.join(KF, 'interf_u_unrouted.kicad_pcb')
PLACED = os.path.join(KF, 'interf_u_unrouted_placed.kicad_pcb')

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def _fill(frame, box, ground):
    """How much of `box` the drawn board spans, on its limiting axis."""
    crop = frame.convert('RGB').crop((box.x, box.y, box.x + box.w,
                                      box.y + box.h))
    mask = Image.new('L', crop.size, 0)
    mp, cp = mask.load(), crop.load()
    for y in range(crop.height):
        for x in range(crop.width):
            if cp[x, y] != ground:
                mp[x, y] = 255
    bb = mask.getbbox()
    if not bb:
        return 0.0
    return max((bb[2] - bb[0]) / float(box.w), (bb[3] - bb[1]) / float(box.h))


def test_the_board_fills_its_box():
    _mark = len(_FAIL)
    ground = RT.theme('dark').rgb('ground')
    worst = (9.0, None)
    # the stage3d frame (the only layout) at every named ratio, and a
    # board-only extreme one
    for rk in ('16:9', '9:16', '1:1', '4:3', '4:1'):
        geo = []
        fr = A.build_boards([('s', ROUTED, None)], ROUTED, 400, 1, None,
                            2, 6, aspect=rk, geom_out=geo, theme='dark',
                            board3d='2d')
        f = _fill(fr[0], geo[0].board, ground)
        if f < worst[0]:
            worst = (f, rk)
        if f < 0.85:
            fail('%s: the board spans %.0f%% of its %dx%d box'
                 % (rk, 100 * f, geo[0].board.w, geo[0].board.h))
    # ...and a CAMERA film ends on the board, not on the pile overview
    import movie_camera as MC
    st = MC.Stage(MC.synth_rounds([SEED, PLACED]), '', tween=3)
    geo = []
    fr = A.build_boards([('a', SEED, None), ('b', PLACED, None)], PLACED,
                        400, 1, None, 2, 6, stage=st,
                        aspect='16:9', geom_out=geo, theme='dark',
                        board3d='2d')
    f = _fill(fr[-1], geo[0].board, ground)
    if f < 0.85:
        fail('the camera film ends with the board at %.0f%% of its box -- '
             'still at the pile overview' % (100 * f))
    if len(_FAIL) == _mark:
        print('  PASS: 5 static frames fill >= %.0f%% (worst %s); the camera '
              'film ends at %.0f%%' % (100 * worst[0], worst[1], 100 * f))


def test_the_title_is_never_a_later_boards_name():
    _mark = len(_FAIL)
    run = os.path.join('C:' if os.name == 'nt' else '/', 'w', 'run32')
    steps = [(n, os.path.join(run, n + '.kicad_pcb'), None)
             for n in ('glasgow_unplaced', 'placed_v2', 'K3C_route')]
    t = A.board_title(steps[-1][1], steps)
    if t != 'run32':
        fail('a one-directory chain is titled %r, not its directory' % t)
    spread = [('a', os.path.join(run, 'a', 'glasgow_unplaced.kicad_pcb'),
               None),
              ('b', os.path.join(run, 'b', 'K3C_route.kicad_pcb'), None)]
    t2 = A.board_title(spread[-1][1], spread)
    if t2 == 'K3C_route':
        fail('a spread chain is titled by its FINAL board')
    if A.board_title(steps[-1][1], steps, hint='My film') != 'My film':
        fail('--title does not win')
    if len(_FAIL) == _mark:
        print('  PASS: %r for one directory, %r for a spread chain' % (t, t2))


def test_layer_cells_have_the_boards_shape_in_a_grid():
    _mark = len(_FAIL)
    asp = 80.0 / 49.0
    tall = FL.Box(0, 0, 360, 560)      # the 1:1 sidebar's strip half
    b, n, gh = RP.grid_boxes(tall, 4, asp)
    rows = len({y for _x, y, _w, _h in b})
    if rows != 2 or n != 4:
        fail('a tall column is not a 2x2 grid: %r' % (b,))
    for x, y, w, h in b:
        if abs(w / float(h - 14) - asp) > 0.05:
            fail('a cell is %dx%d, not the board shape %.2f' % (w, h - 14,
                                                              asp))
            break
    wide = FL.Box(0, 0, 1380, 230)
    b2, _n, _g = RP.grid_boxes(wide, 4, asp)
    if len({y for _x, y, _w, _h in b2}) != 1:
        fail('a wide box is not one row: %r' % (b2,))
    xs = [x for x, _y, _w, _h in b]
    left = min(xs) - tall.x
    right = tall.x + tall.w - max(x + w for x, _y, w, _h in b)
    if abs(left - right) > 2:
        fail('the grid is not centred: %d px left, %d right' % (left, right))
    if len(_FAIL) == _mark:
        print('  PASS: 2x2 in a tall column, one row in a wide box, cells '
              '%.2f:1 like the board, centred' % asp)


class _Rec(object):
    def __init__(self, d):
        self._d, self.boxes = d, []

    def text(self, xy, txt, *a, **kw):
        self.boxes.append((txt, self._d.textbbox(
            xy, txt, font=kw.get('font'), anchor=kw.get('anchor'))))
        return self._d.text(xy, txt, *a, **kw)

    def __getattr__(self, k):
        return getattr(self._d, k)


class _R(object):
    """The renderer surface `_draw_panel` reads."""

    def __init__(self, pcb):
        self.theme = RT.theme('dark')
        self.pcb = pcb
        self.bounds = pcb.board_info.board_bounds
        self.copper_layers = list(pcb.board_info.copper_layers)
        self.palette = {}


def test_every_panel_keeps_the_gutter():
    _mark = len(_FAIL)
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(ROUTED)
    r = _R(pcb)
    n = 0
    for rk in ('4:3', '1:1', '16:9', '9:16'):
        lk = 'stage3d'
        g = FL.plan_frame(r.bounds, ratio=FL.parse_ratio(rk), size=1400)
        gut = RC.gutter_px(g.frame.w)
        inner = g.panel
        if inner is None:
            fail('%s: the stage3d frame dropped its layer column' % rk)
            continue
        # one content on every frame -- the layer strip and the board's
        # numbers -- whatever the event
        for event in ('moving 5 part(s)', 'route', 'input'):
            im = Image.new('RGB', (g.frame.w, g.frame.h))
            rec = _Rec(ImageDraw.Draw(im))
            A._draw_panel(rec, g, r, {'event': event, 'live': (),
                                      'live_v': ()})
            for txt, bb in rec.boxes:
                n += 1
                if (bb[0] < inner.x + gut - 1 or bb[1] < inner.y + gut - 1
                        or bb[2] > inner.x + inner.w - gut + 1
                        or bb[3] > inner.y + inner.h - gut + 1):
                    fail('%s/%s %s: %r at %r is inside the %d px gutter of '
                         '%r' % (lk, rk, event, txt, bb, gut, tuple(inner)))
                    break
    if len(_FAIL) == _mark:
        print('  PASS: %d panel texts over 4 ratios x 3 events, all '
              'inside the gutter' % n)


def test_the_event_key_is_in_the_rail_not_on_the_board():
    _mark = len(_FAIL)
    m = A.Movie.__new__(A.Movie)
    m.seen_events = ['event_new', 'event_ripped']
    m.theme = RT.theme('dark')
    m.split_caption = True
    if m._key_overlay() is not None:
        fail('a layout with a rail still draws the key over the board')
    m.split_caption = False
    if m._key_overlay() is None:
        fail('a frame with no rail (build_single) lost its key')
    rail = FL.Box(0, 0, 1400, 36)
    im = Image.new('RGB', (1400, 60))
    rec = _Rec(ImageDraw.Draw(im))
    RC.draw_rail(rec, rail, 'run32', 'K3C_route', theme=RT.theme('dark'),
                 key_rows=RC.event_rows(RT.theme('dark'),
                                        seen=['event_new', 'event_ripped']))
    keyed = [bb for t, bb in rec.boxes if t in ('new', 'ripped')]
    if len(keyed) != 2:
        fail('the rail did not draw the key: %r' % rec.boxes)
    for bb in keyed:
        if bb[3] > rail.y + rail.h:
            fail('a key label leaves the rail: %r' % (bb,))
    if len(_FAIL) == _mark:
        print('  PASS: key in the rail on a railed frame; corner key kept '
              'for a frame with no rail')


TESTS = (
    test_the_board_fills_its_box,
    test_the_title_is_never_a_later_boards_name,
    test_layer_cells_have_the_boards_shape_in_a_grid,
    test_every_panel_keeps_the_gutter,
    test_the_event_key_is_in_the_rail_not_on_the_board,
)


def main():
    for fn in TESTS:
        print('%s:' % fn.__name__)
        fn()
    if _FAIL:
        print('')
        print('%d FAILURE(S)' % len(_FAIL))
        for m_ in _FAIL:
            print('  - %s' % m_)
        return 1
    print('')
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
