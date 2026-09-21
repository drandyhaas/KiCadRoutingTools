#!/usr/bin/env python3
"""The caption splits into rail / event / totals (#946 item 12, #1019).

`route_render._label` stamped ONE top-left strip that was simultaneously title,
step indicator, event name and metrics readout. Its own docstring records what
that cost:

    "Measured on a 217-part board: the caption built 156 chars and ~117 fit, so
    `hole-conflict 0.60mm` and `oob 7` -- a fab blocker and the off-board count
    ... were absent from the picture while the strip looked complete because it
    ended at a plausible-looking field."

Wrapping fixed the clipping and treated the symptom. **One strip doing four
jobs is why it overflowed**, and wrapping then cost vertical space on every
frame to serve the worst case.

The claim this file makes is not "the caption is prettier". It is:

  * **the 156-char case keeps every field**, because each region is sized for
    its own content, so a long event line can no longer push the totals off the
    edge of a strip that still looks complete;
  * **the rail counts LAPS, not steps** -- a loop revisits the same step, so
    `step 2` cannot say whether this is the first attempt or the fourth;
  * **the over-board strip is gone when a rail carries it**, because a
    duplicate sitting on the copper is worse than no strip at all.
"""
import os
import re
import sys

RUN_ALL_FAST_OK = True

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from PIL import Image, ImageChops, ImageDraw
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import frame_layout as FL       # noqa: E402
import render_chrome as RC      # noqa: E402
import render_theme as RT       # noqa: E402

#: The measured case, rebuilt from the docstring that records it.
LONG_EVENT = ('step2 route  142/190  reroute /DDR_A7_BANK1_LOWER  rip 3  '
              'blocked by /VCCIO_1V8 and /GND_ANALOG_RETURN')
TOTALS = 'drc 2 | oob 7 | hole-conflict 0.60mm'

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def test_the_156_char_case_keeps_every_field():
    """The claim, measured: no field is dropped because another was long."""
    _mark = len(_FAIL)
    th = RT.DARK
    g = FL.plan_frame((0, 0, 185, 100), layout='sidebar', size=900,
                      panel=True)
    img = Image.new('RGB', (g.frame.w, g.frame.h), th.rgb('ground'))
    d = ImageDraw.Draw(img)
    RC.draw_rail(d, g.rail, 'a_217_part_board', 'lap 3 of 5  -  route',
                 theme=th, progress=0.6, ticks=(0.0, 0.3, 0.6))
    d.rectangle([g.foot.x, g.foot.y, g.foot.x + g.foot.w - 1,
                 g.foot.y + g.foot.h - 1], fill=th.rgb('chrome_panel'))
    drawn = RC.draw_totals(d, g.foot, TOTALS, theme=th)

    # Every field must be REPRESENTED in what was drawn -- stacked on its own
    # line when the region can hold every field, joined onto one line when it
    # cannot. A region running out of rows must not reintroduce the dropped
    # field by a different route.
    blob = ' '.join(drawn)
    missing = [p.strip() for p in TOTALS.split('|')
               if p.strip() and p.strip() not in blob]
    if missing:
        fail('%d field(s) dropped: %s  (drew %r)'
             % (len(missing), ', '.join(missing), drawn))
    else:
        print('    all %d fields present in %d drawn line(s): %r'
              % (len(TOTALS.split('|')), len(drawn), drawn))

    # and the event line must not be able to eat the totals' room: they are in
    # DIFFERENT boxes, so assert exactly that
    if g.rail.overlaps(g.foot):
        fail('the rail and the foot overlap -- they are not separate regions')
    if len(_FAIL) == _mark:
        print('  PASS: each region is sized for its own content')


def test_a_long_line_ellipsises_inside_its_own_region():
    _mark = len(_FAIL)
    th = RT.DARK
    g = FL.plan_frame((0, 0, 185, 100), layout='sidebar', size=700, panel=True)
    img = Image.new('RGB', (g.frame.w, g.frame.h), th.rgb('ground'))
    d = ImageDraw.Draw(img)
    RC.draw_rail(d, g.rail, LONG_EVENT, 'lap 1 of 1', theme=th)
    # nothing outside the rail may have been touched
    below = img.crop((0, g.rail.y + g.rail.h, g.frame.w, g.frame.h))
    cols = {c for _n, c in below.convert('RGB').getcolors(1 << 20)}
    if cols != {th.rgb('ground')}:
        fail('the rail drew outside its own box: %s' % sorted(cols)[:4])
    else:
        print('    a %d-char line stayed inside a %d px rail'
              % (len(LONG_EVENT), g.rail.h))
    from route_render import load_font
    font = load_font(14)
    fitted = RC._fit(d, LONG_EVENT, font, 120)
    if not fitted.endswith('...'):
        fail('a too-long line was not ellipsised: %r' % fitted)
    if len(_FAIL) == _mark:
        print('  PASS: each region clips itself, and says it clipped')


def test_the_rail_counts_laps_not_steps():
    _mark = len(_FAIL)
    if RC.lap_text('round 2 routed', lap=3, laps=5, phase='route') \
            != 'lap 3 of 5  -  route':
        fail('lap_text: %r' % RC.lap_text('round 2 routed', lap=3, laps=5,
                                          phase='route'))
    # no loop -> no laps to count, and the step label is the honest thing
    if RC.lap_text('step1 route') != 'step1 route':
        fail('a non-loop chain invented a lap: %r' % RC.lap_text('step1 route'))
    import animate_route as A
    if A.render_chrome_lap(3, [1, 2, 3], 'round 3 routed') != \
            'lap 3 of 3  -  route':
        fail('render_chrome_lap: %r'
             % A.render_chrome_lap(3, [1, 2, 3], 'round 3 routed'))
    if A.render_chrome_lap(9, [1, 2, 3], 'round 9') != 'round 9':
        fail('an unknown round was not passed through unchanged')
    if len(_FAIL) == _mark:
        print('  PASS: lap N of M when there is a loop, the step label when '
              'there is not')


def test_the_over_board_strip_is_gone_when_a_rail_carries_it():
    """Asserted on the FRAMES, not on the source.

    The first version of this grepped `animate_route.py` for the exact
    expression `split_caption = bool(geom_out and geom_out[0].rail.h ...)`. It
    went red on a pure RENAME -- `geom_out` became a private `_geom` when
    `geom_out` stopped being the switch and went back to being only an output
    collector -- while the behaviour it names was unchanged. A source grep
    pins a spelling; this pins the thing the spelling is for.

    The claim: a caption drawn over the copper AND repeated in the rail is a
    duplicate, and a duplicate sitting on the board is worse than no strip at
    all. So the over-board strip exists on `legacy` (no rail) and not on a
    layout that reserves one.
    """
    _mark = len(_FAIL)
    import animate_route as A
    import shutil
    import tempfile
    from kicad_parser import parse_kicad_pcb
    board = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
    with tempfile.TemporaryDirectory() as td:
        b = os.path.join(td, 'step2_x.kicad_pcb')
        shutil.copyfile(board, b)
        steps = [('step1 route', board, None), ('step2 route', b, None)]
        ink = {}
        for layout in ('legacy', 'split'):
            m = A.build_boards(steps, b, 300, 1, 150, 2, 3, layout=layout)
            if not m:
                fail('%s: no frames' % layout)
                continue
            # `_label` stamps the strip in `chrome_strip` at the TOP LEFT of
            # the BOARD; count that colour in the first 30 rows of the board
            # box, which on legacy is the whole frame's top.
            import render_theme as _rt
            strip = _rt.DARK.rgb('chrome_band')   # _label's own box fill
            f = m[len(m) // 2].convert('RGB')
            top = f.crop((0, 0, f.width, min(30, f.height)))
            ink[layout] = sum(n for n, c in top.getcolors(1 << 20)
                              if c == strip)
        if 'legacy' in ink and 'split' in ink:
            if not ink['legacy']:
                fail('BROKEN TEST: legacy drew no over-board strip at all, so '
                     'this cannot tell suppression from an empty frame')
            elif ink['split'] >= ink['legacy']:
                fail('the over-board strip survives on a layout WITH a rail: '
                     'legacy %d px vs split %d px -- that is a duplicate '
                     'sitting on the copper' % (ink['legacy'], ink['split']))
            else:
                print('    over-board strip: legacy %d px, split %d px'
                      % (ink['legacy'], ink['split']))
    # and the switch is still conditional on a rail EXISTING, by name
    src = open(os.path.join(ROOT, 'py_router', 'animate_route.py'),
               encoding='utf-8').read()
    if 'split_caption' not in src:
        fail('nothing suppresses the over-board caption')
    if not re.search(r'split_caption\s*=\s*bool\([^)]*\.rail\.h', src):
        fail('the suppression is not conditional on a rail existing')
    if len(_FAIL) == _mark:
        print('  PASS: suppressed only when a rail exists; legacy keeps its '
              'strip')


TESTS = (
    test_the_156_char_case_keeps_every_field,
    test_a_long_line_ellipsises_inside_its_own_region,
    test_the_rail_counts_laps_not_steps,
    test_the_over_board_strip_is_gone_when_a_rail_carries_it,
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
