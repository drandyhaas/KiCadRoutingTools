#!/usr/bin/env python3
"""A crossing must not impersonate a third layer (#946 item 9, #1015).

`tests/test_946_palette_measures.py` measures the PALETTE: given the blend
`frame()` used to perform, 19 two-layer crossings land within 34 of some third
layer's solo appearance, the worst at 5.1. That number is a property of ten
colours and an alpha, and it does not change when the renderer stops blending.

This measures THE RENDERED IMAGE, which is the thing a viewer actually sees.
It renders a real multi-layer board and counts distinct colours that sit within
the collision threshold of a solo layer colour without BEING one -- i.e. blends
that impersonate.

**THE OFF ARM IS THE POINT.** A test that only asserts "zero impersonating
blends" passes just as happily on a blank image, on a board with one layer, or
on a renderer that silently stopped drawing copper. The control renders the
same board with `opaque_crossings=False` and requires the count to be ABOVE
zero -- so the measurement is shown to be capable of detecting the defect
before it is used to claim the defect is gone.

Also pins `focus_layer`: the layer the current event is on draws at full
strength while the rest drop to context. The movie already knows which layer
each event touches, so the focus costs nothing -- and a crossing stops being
ambiguous because only one side of it is lit.
"""
import os
import sys

RUN_ALL_TIMEOUT = 300

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from PIL import Image  # noqa: F401
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import palette_audit as PA       # noqa: E402
import render_theme as RT        # noqa: E402
from kicad_parser import parse_kicad_pcb   # noqa: E402
from route_render import BoardRenderer     # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
THRESHOLD = 34.0

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def legitimate_layer_colours(theme):
    """Every appearance a layer is ALLOWED to have.

    Two per layer, and the second one cost a debugging round to learn. A layer
    composited over the board body at `layer_alpha` is the ordinary wash. A
    layer drawn OPAQUE at a crossing is the #1015 fix doing its job -- and it
    renders as the RAW palette colour, which is nowhere near the translucent
    one.

    The first version of this test listed only the translucent appearance and
    flagged `(158,141,58)` on the light arm as an impersonating blend. That
    colour is exactly light `In2` at full alpha: the measurement was wrong, not
    the renderer. A measurement that does not know what a correct result looks
    like will report the fix as the defect.
    """
    body = theme.rgb('board_body')
    out = []
    for c in theme.layers:
        out.append(PA.composite(c, body, theme.layer_alpha))
        out.append(tuple(c))
    # And every DECLARED structural colour. These are not layers and cannot
    # impersonate one -- they are the substrate the layers are drawn on. The
    # light pad (150,118,24) sits inside the threshold of a layer appearance,
    # which is a palette fact worth knowing and not a compositing artefact;
    # this test is about colours the renderer INVENTED.
    for role in ('ground', 'board_body', 'board_edge', 'pad', 'pad_hole',
                 'via', 'via_hole', 'zone_tint'):
        out.append(tuple(theme.rgb(role)))
    return out


def _impersonating(img, theme):
    """Distinct rendered colours that look like a layer they are not."""
    ok = legitimate_layer_colours(theme)
    out = set()
    for _n, c in img.convert('RGB').getcolors(1 << 22):
        near = min(PA.rgb_distance(c, s) for s in ok)
        if near < THRESHOLD and all(PA.rgb_distance(c, s) > 1.0 for s in ok):
            out.add(c)
    return out


def _render(theme, **kw):
    pcb = parse_kicad_pcb(BOARD)
    r = BoardRenderer(pcb, size=500, supersample=1, theme=theme)
    return r, r.frame(segments=pcb.segments, vias=[], **kw)


def test_the_measurement_can_detect_the_defect():
    """THE CONTROL. Without this, zero proves nothing."""
    _mark = len(_FAIL)
    for name in sorted(RT.THEMES):
        th = RT.theme(name)
        _r, img = _render(th, opaque_crossings=False)
        n = len(_impersonating(img, th))
        if n <= 0:
            fail('the %s arm shows %d impersonating blends with crossings '
                 'BLENDED -- the measurement cannot see the defect, so a zero '
                 'from it would prove nothing' % (name, n))
        else:
            print('    %-6s blended:  %2d impersonating blend(s)' % (name, n))
    if len(_FAIL) == _mark:
        print('  PASS: the control reproduces the defect on every arm')


def test_opaque_crossings_remove_them():
    _mark = len(_FAIL)
    for name in sorted(RT.THEMES):
        th = RT.theme(name)
        _r, img = _render(th, opaque_crossings=True)
        bad = _impersonating(img, th)
        if bad:
            fail('the %s arm still shows %d impersonating blend(s): %s'
                 % (name, len(bad), sorted(bad)[:4]))
        else:
            print('    %-6s opaque:    0 impersonating blends' % name)
    if len(_FAIL) == _mark:
        print('  PASS: no rendered colour impersonates a layer it is not')


def test_it_is_the_default():
    """A fix nobody turns on is not a fix."""
    pcb = parse_kicad_pcb(BOARD)
    r = BoardRenderer(pcb, size=120, supersample=1)
    if not getattr(r, 'opaque_crossings', False):
        fail('opaque_crossings is not on by default')
        return
    print('  PASS: on by default, overridable per frame()')


def test_focus_lights_one_layer_and_dims_the_rest():
    _mark = len(_FAIL)
    th = RT.DARK
    pcb = parse_kicad_pcb(BOARD)
    r = BoardRenderer(pcb, size=400, supersample=1, theme=th)
    layers = [ln for ln in r.copper_layers
              if any(s.layer == ln for s in pcb.segments)]
    if len(layers) < 2:
        fail('BROKEN: the fixture board has %d drawn layer(s); this test '
             'needs at least 2 to say anything' % len(layers))
        return
    plain = r.frame(segments=pcb.segments, vias=[])
    focused = r.frame(segments=pcb.segments, vias=[], focus_layer=layers[0])
    if plain.tobytes() == focused.tobytes():
        fail('focus_layer changed nothing')
        return
    # the focused layer must not be the thing that got dimmer
    body = th.rgb('board_body')
    lit = PA.composite(r.palette[layers[0]], body, th.layer_alpha)
    ctx = PA.composite(r.palette[layers[1]], body,
                       max(1, int(round(th.layer_alpha * r.context_alpha_frac))))
    have = {c for _n, c in focused.convert('RGB').getcolors(1 << 22)}
    near_lit = min(PA.rgb_distance(lit, c) for c in have)
    near_ctx = min(PA.rgb_distance(ctx, c) for c in have)
    if near_lit > 8:
        fail('the focused layer %s is not drawn at full strength (nearest '
             'rendered colour is %.1f away)' % (layers[0], near_lit))
    if near_ctx > 8:
        fail('the context layer %s is not drawn at the context alpha '
             '(nearest is %.1f away)' % (layers[1], near_ctx))
    if len(_FAIL) == _mark:
        print('  PASS: %s lit, %s at %.0f%% context'
              % (layers[0], layers[1], r.context_alpha_frac * 100))


TESTS = (
    test_the_measurement_can_detect_the_defect,
    test_opaque_crossings_remove_them,
    test_it_is_the_default,
    test_focus_lights_one_layer_and_dims_the_rest,
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
