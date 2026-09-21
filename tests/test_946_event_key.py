#!/usr/bin/env python3
"""A key in the frame, drawn from the events the run ACTUALLY produced
(#946 item 8, #1014).

`route_render` named its event colours in three modules and put none of them on
screen, so a viewer had to be told what red meant. The key fixes that -- but the
rule that makes a key honest is not "draw a key", it is the one
`py_tools/render_placement.draw_legend` has had right since #896 and had alone:

    Only the keys the panel can ACTUALLY SHOW are drawn -- a legend listing
    arrows on a panel with no --before is itself misinformation.

That rule is if anything more useful on the routing side, because **a movie of a
clean run never rips anything and must not advertise a rip colour**.

What this file pins:

  * **a clean run's key has no rip row**, and a run that ripped has one. `seen`
    is not optional on the movie path;
  * **the key and the draw site cannot disagree**, because `rows_for_roles`
    reads BOTH the colour and the mark off the same theme -- a role drawn
    dashed is keyed dashed, because neither side chooses;
  * **the key grows with the film.** At frame N it says what has happened by
    frame N, never what is coming -- which is the difference between a key and
    a spoiler;
  * **a key never fails a render.** `draw_legend` wraps its whole body in
    `except Exception: pass` with the comment "a legend is never worth failing
    a render over", and that is kept. A movie is an artifact; losing one to a
    font metric trades a cosmetic problem for a real one;
  * **it draws.** A function wrapped in a bare `except` returns None whether it
    drew or not, so the ink is counted.
"""
import os
import sys

RUN_ALL_FAST_OK = True

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from PIL import Image, ImageDraw
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import render_chrome as RC        # noqa: E402
import render_theme as RT         # noqa: E402

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def test_a_clean_run_advertises_no_rip():
    _mark = len(_FAIL)
    clean = RC.event_rows(RT.DARK, seen=['event_new'])
    ripped = RC.event_rows(RT.DARK, seen=['event_new', 'event_ripped'])
    if [t for _c, _m, t in clean] != ['new']:
        fail('a clean run keyed %s' % [t for _c, _m, t in clean])
    if 'ripped' not in [t for _c, _m, t in ripped]:
        fail('a run that ripped did NOT key the rip: %s'
             % [t for _c, _m, t in ripped])
    # `seen=None` is the vocabulary-documenting still, not a run report
    every = RC.event_rows(RT.DARK, seen=None)
    if len(every) != len(RC.EVENT_KEY):
        fail('seen=None keyed %d of %d roles' % (len(every), len(RC.EVENT_KEY)))
    if len(clean) >= len(every):
        fail('the filter is inert: clean=%d, all=%d' % (len(clean),
                                                        len(every)))
    print('    clean %s   ripped %s   vocabulary %s'
          % ([t for _c, _m, t in clean], [t for _c, _m, t in ripped],
             [t for _c, _m, t in every]))
    if len(_FAIL) == _mark:
        print('  PASS: the key reports the run, not the palette')


def test_the_key_and_the_draw_site_cannot_disagree():
    """Both read the mark off the theme, so neither chooses."""
    _mark = len(_FAIL)
    for th in (RT.DARK, RT.theme('light')):
        for role, label in RC.EVENT_KEY:
            rows = RC.rows_for_roles(th, [(role, label)])
            rgb, mark, text = rows[0]
            if rgb != th.rgb(role):
                fail('%s/%s: key colour %s != theme %s'
                     % (th.name, role, rgb, th.rgb(role)))
            if mark != th.mark(role):
                fail('%s/%s: key mark %r != theme %r'
                     % (th.name, role, mark, th.mark(role)))
            if text != label:
                fail('%s/%s: label %r' % (th.name, role, text))
        # the rip is the one that MUST carry a second channel -- #1013's whole
        # point is that a colour alone is not enough for it
        if th.mark('event_ripped') == 'solid':
            fail('%s: the rip has no second channel' % th.name)
    if len(_FAIL) == _mark:
        print('  PASS: one source for the colour and the mark')


def test_the_key_grows_with_the_film():
    _mark = len(_FAIL)
    import animate_route as A
    from kicad_parser import parse_kicad_pcb
    board = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
    pcb = parse_kicad_pcb(board)
    rows_s, rows_v = A._board_rows(pcb, list(pcb.board_info.copper_layers))
    r, ls = A._renderer(board, None, 160, 1, 150)
    m = A.Movie(r, ls, rip_hold=1)
    if m.seen_events:
        fail('a fresh movie already claims events: %s' % m.seen_events)
    m.add(rows_s[:40], [], 'route', 'new')
    after_add = list(m.seen_events)
    if after_add != ['event_new']:
        fail('after one plain add the key says %s' % after_add)
    m.reveal_delta(rows_s[:20], rows_v, 'reroute')
    if 'event_ripped' not in m.seen_events:
        fail('a rip did not reach the key: %s' % m.seen_events)
    if m.seen_events.index('event_new') != 0:
        fail('the key is not in the order the viewer met the events: %s'
             % m.seen_events)
    print('    after add %s -> after rip %s' % (after_add, m.seen_events))
    if len(_FAIL) == _mark:
        print('  PASS: at frame N it says what happened by frame N')


def test_it_draws_and_it_never_raises():
    _mark = len(_FAIL)
    bg = (7, 7, 7)
    img = Image.new('RGB', (420, 300), bg)
    rows = RC.event_rows(RT.DARK, seen=None)
    RC.draw_key(ImageDraw.Draw(img), rows, width=420, height=300,
                theme=RT.DARK, corner='bl')
    cols = {c for _n, c in img.getcolors(1 << 20)}
    if len(cols) < 4:
        fail('the key drew %d colour(s) -- it swallowed an exception'
             % len(cols))
    for rgb, _m, _t in rows:
        if rgb not in cols:
            fail('%s never reached the canvas' % (rgb,))
    # no rows -> nothing drawn, which is the honest empty state
    blank = Image.new('RGB', (420, 300), bg)
    RC.draw_key(ImageDraw.Draw(blank), [], width=420, height=300,
                theme=RT.DARK)
    if {c for _n, c in blank.getcolors(1 << 20)} != {bg}:
        fail('an empty key still put ink on the canvas')
    # and a broken call must not take the render down
    for bad in ((None, 420, 300), (rows, 0, 0), (rows, -5, -5)):
        try:
            RC.draw_key(ImageDraw.Draw(Image.new('RGB', (40, 40), bg)),
                        bad[0] if bad[0] is not None else [(None, 'nope', 1)],
                        width=bad[1], height=bad[2], theme=RT.DARK)
        except Exception as exc:                               # noqa: BLE001
            fail('a key raised %s -- a movie is an artifact and must not be '
                 'lost to a font metric' % exc.__class__.__name__)
    print('    %d colours drawn, %d key rows, empty draws nothing'
          % (len(cols), len(rows)))
    if len(_FAIL) == _mark:
        print('  PASS: it draws, and it never takes the film down')


def test_every_corner_stays_inside_the_canvas():
    _mark = len(_FAIL)
    bg = (7, 7, 7)
    rows = RC.event_rows(RT.DARK, seen=None)
    for corner in ('bl', 'br', 'tl', 'tr'):
        img = Image.new('RGB', (400, 260), bg)
        RC.draw_key(ImageDraw.Draw(img), rows, width=400, height=260,
                    theme=RT.DARK, corner=corner)
        px = img.convert('RGB')
        ink = [(x, y) for y in range(0, 260, 2) for x in range(0, 400, 2)
               if px.getpixel((x, y)) != bg]
        if not ink:
            fail('%s drew nothing' % corner)
            continue
        xs = [p[0] for p in ink]
        ys = [p[1] for p in ink]
        want_right = corner in ('br', 'tr')
        want_bottom = corner in ('bl', 'br')
        if want_right and min(xs) < 200:
            fail('%s: ink reaches x=%d, which is not the right corner'
                 % (corner, min(xs)))
        if not want_right and max(xs) > 300:
            fail('%s: ink reaches x=%d' % (corner, max(xs)))
        if want_bottom and min(ys) < 120:
            fail('%s: ink reaches y=%d, which is not the bottom'
                 % (corner, min(ys)))
        if not want_bottom and max(ys) > 180:
            fail('%s: ink reaches y=%d' % (corner, max(ys)))
    if len(_FAIL) == _mark:
        print('  PASS: bl / br / tl / tr each land in their own corner')


TESTS = (
    test_a_clean_run_advertises_no_rip,
    test_the_key_and_the_draw_site_cannot_disagree,
    test_the_key_grows_with_the_film,
    test_it_draws_and_it_never_raises,
    test_every_corner_stays_inside_the_canvas,
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
