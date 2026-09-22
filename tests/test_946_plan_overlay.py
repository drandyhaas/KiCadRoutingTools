#!/usr/bin/env python3
"""The declared floorplan is drawn, and lands where it was declared (#946 item
5, #1017).

A floorplan intent is a fully geometric document -- `blocks[].zone`,
`keepouts[].rect`/`circle`, `edge_connectors[].along_edge_band`. Nothing drew
it: `render_placement --intent` read that file for exactly ONE thing,
`overlap_waivers`, so the plan existed only as JSON and as pass/fail counts.

Three things pinned here, and the third is the one that matters most.

  * A declared rect lands at `renderer.tf.pt()` of its own world coordinates.
    "It drew something" is not the claim; "it drew it THERE" is.
  * **NEGATIVE CONTROL.** An intent with nothing declared must SAY so, because
    a picture of nothing is indistinguishable from a picture of a plan that was
    met. A test that only checks the populated case passes just as well on a
    renderer that silently draws nothing.
  * It costs no frame geometry, because it goes through `frame(overlays=...)`.
"""
import io
import json
import os
import subprocess
import sys
import tempfile

RUN_ALL_TIMEOUT = 300

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_tools'), os.path.join(ROOT, 'py_placer')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from PIL import Image, ImageChops, ImageDraw
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import render_plan as RPL                       # noqa: E402
import render_theme as RT                       # noqa: E402
from kicad_parser import parse_kicad_pcb        # noqa: E402
from route_render import BoardRenderer          # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def _intent_doc(pcb, blocks=True):
    x0, y0, x1, y1 = pcb.board_info.board_bounds
    W, H = x1 - x0, y1 - y0
    doc = {'schema': 1, 'kind': 'floorplan-intent',
           'board': os.path.basename(BOARD), 'units': 'mm',
           'envelope': {'rect': [x0, y0, x1, y1]},
           'blocks': [], 'keepouts': [], 'edge_connectors': []}
    if blocks:
        doc['blocks'] = [{'name': 'mcu',
                          'zone': [x0 + W * 0.30, y0 + H * 0.26,
                                   x0 + W * 0.70, y0 + H * 0.72],
                          'refs': ['U1']}]
        doc['keepouts'] = [{'name': 'mount',
                            'rect': [x0 + W * 0.03, y0 + H * 0.70,
                                     x0 + W * 0.16, y0 + H * 0.95]}]
        doc['edge_connectors'] = [{'ref': 'J1', 'edge': 'west',
                                   'along_edge_band': {'from': 0.2,
                                                       'to': 0.6}}]
    return doc


def _write(doc):
    fh = tempfile.NamedTemporaryFile('w', suffix='.intent.json',
                                     delete=False, encoding='utf-8')
    json.dump(doc, fh)
    fh.close()
    return fh.name


def test_a_declared_rect_lands_where_it_was_declared():
    _mark = len(_FAIL)
    from placement.floorplan import load_intent
    pcb = parse_kicad_pcb(BOARD)
    doc = _intent_doc(pcb)
    path = _write(doc)
    try:
        it = load_intent(path)
    finally:
        os.unlink(path)
    r = BoardRenderer(pcb, size=700, supersample=1)
    plain = r.frame(segments=[], vias=[])
    drawn = r.frame(segments=[], vias=[], overlays=[RPL.plan_overlay(it)])
    diff = ImageChops.difference(plain, drawn)
    bb = diff.getbbox()
    if bb is None:
        fail('the plan overlay drew nothing at all')
        return
    # the zone's own corners, through the renderer's transform
    zx0, zy0, zx1, zy1 = it.blocks[0].rect
    p0, p1 = r.tf.pt(zx0, zy0), r.tf.pt(zx1, zy1)
    want = (min(p0[0], p1[0]), min(p0[1], p1[1]),
            max(p0[0], p1[0]), max(p0[1], p1[1]))
    # the changed region must CONTAIN the declared rect, within a few px of
    # stroke width and label ascent
    slack = 24
    if not (bb[0] <= want[0] + slack and bb[1] <= want[1] + slack
            and bb[2] >= want[2] - slack and bb[3] >= want[3] - slack):
        fail('the drawn region %s does not contain the declared rect %s'
             % (bb, tuple(round(v) for v in want)))
        return
    print('    declared %s -> drawn within %s'
          % (tuple(round(v) for v in want), bb))
    if plain.size != drawn.size:
        fail('the frame size moved: %s -> %s' % (plain.size, drawn.size))
    if len(_FAIL) == _mark:
        print('  PASS: the plan lands at tf.pt() of its own coordinates, and '
              'the frame size is unchanged')


def test_the_keepout_hatch_stays_inside_its_own_box():
    """A keep-out is a PROHIBITION, and a prohibition drawn bigger than it was
    declared is worse than one drawn in the wrong colour.

    The hatch is 45-degree diagonals across the rect, and the first version
    drew the FULL diagonals -- they ran out past the keep-out on both sides.
    The clip is the fix; this is the check that can see it. Asserted on the
    keep-out's own colour, so the block zone and the edge band cannot mask a
    leak.
    """
    _mark = len(_FAIL)
    from placement.floorplan import load_intent
    import render_theme as RT
    pcb = parse_kicad_pcb(BOARD)
    doc = _intent_doc(pcb)
    # ONLY the keep-out: a rect elsewhere in the picture would make "is this
    # pixel outside the keep-out" a question about the wrong shape.
    doc['blocks'] = []
    doc['edge_connectors'] = []
    path = _write(doc)
    try:
        it = load_intent(path)
    finally:
        os.unlink(path)
    r = BoardRenderer(pcb, size=700, supersample=1)
    drawn = r.frame(segments=[], vias=[],
                    overlays=[RPL.plan_overlay(it)]).convert('RGB')
    keep = RT.default_theme().rgb('defect_courtyard')
    _k0 = it.keepouts[0]
    kx0, ky0, kx1, ky1 = (_k0.get('rect') if isinstance(_k0, dict)
                          else _k0.rect)
    p0, p1 = r.tf.pt(kx0, ky0), r.tf.pt(kx1, ky1)
    bx0, by0 = min(p0[0], p1[0]), min(p0[1], p1[1])
    bx1, by1 = max(p0[0], p1[0]), max(p0[1], p1[1])
    inside = outside = 0
    worst = None
    W, H = drawn.size
    for y in range(H):
        for x in range(W):
            if drawn.getpixel((x, y)) != keep:
                continue
            # 2 px of slack for the outline's own stroke width
            if (bx0 - 2 <= x <= bx1 + 2) and (by0 - 2 <= y <= by1 + 2):
                inside += 1
            else:
                outside += 1
                d = max(bx0 - x, x - bx1, by0 - y, y - by1)
                if worst is None or d > worst[0]:
                    worst = (d, x, y)
    if not inside:
        fail('BROKEN TEST: the keep-out drew no ink in its own colour, so this '
             'cannot tell a leak from a blank picture')
        return
    if outside:
        fail('%d keep-out pixel(s) outside the declared rect, worst %.0f px '
             'out at %s -- a prohibition drawn bigger than it was declared'
             % (outside, worst[0], (worst[1], worst[2])))
    else:
        print('    %d px of hatch, 0 outside the %.0fx%.0f px rect'
              % (inside, bx1 - bx0, by1 - by0))
    if len(_FAIL) == _mark:
        print('  PASS: the hatch is clipped to the rule it draws')


def test_an_empty_intent_says_it_drew_nothing():
    """THE NEGATIVE CONTROL. A picture of nothing is indistinguishable from a
    picture of a plan that was met."""
    _mark = len(_FAIL)
    from placement.floorplan import load_intent
    pcb = parse_kicad_pcb(BOARD)
    path = _write(_intent_doc(pcb, blocks=False))
    try:
        it = load_intent(path)
    finally:
        os.unlink(path)
    line = RPL.plan_summary(it)
    if 'NOTHING DECLARED' not in line:
        fail('an empty intent does not announce itself: %r' % line)
    else:
        print('    %s' % line)
    r = BoardRenderer(pcb, size=400, supersample=1)
    plain = r.frame(segments=[], vias=[])
    drawn = r.frame(segments=[], vias=[], overlays=[RPL.plan_overlay(it)])
    if ImageChops.difference(plain, drawn).getbbox() is not None:
        fail('an empty intent drew something')
    if len(_FAIL) == _mark:
        print('  PASS: nothing declared, nothing drawn, and it says so')


def test_the_verdict_recolours_held_and_drifted():
    _mark = len(_FAIL)
    from placement.floorplan import load_intent
    pcb = parse_kicad_pcb(BOARD)
    path = _write(_intent_doc(pcb))
    try:
        it = load_intent(path)
    finally:
        os.unlink(path)
    r = BoardRenderer(pcb, size=520, supersample=1)
    th = r.theme
    held = r.frame(segments=[], vias=[],
                   overlays=[RPL.plan_overlay(it, verdict={'mcu': True})])
    drift = r.frame(segments=[], vias=[],
                    overlays=[RPL.plan_overlay(it, verdict={'mcu': False})])
    if held.tobytes() == drift.tobytes():
        fail('held and drifted render identically')
        return
    hc = {c for _n, c in held.convert('RGB').getcolors(1 << 22)}
    dc = {c for _n, c in drift.convert('RGB').getcolors(1 << 22)}
    if th.rgb('status_kept') not in hc:
        fail('a held block is not drawn in status_kept')
    if th.rgb('defect_conflict') not in dc:
        fail('a drifted block is not drawn in defect_conflict')
    if len(_FAIL) == _mark:
        print('  PASS: held -> status_kept, drifted -> defect_conflict')


def test_render_placement_announces_the_plan_either_way():
    _mark = len(_FAIL)
    pcb = parse_kicad_pcb(BOARD)
    env = dict(os.environ)
    env['PYTHONPATH'] = os.pathsep.join(
        [os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_tools'),
         os.path.join(ROOT, 'py_placer'), env.get('PYTHONPATH', '')])
    tool = os.path.join(ROOT, 'py_tools', 'render_placement.py')
    for blocks, want in ((True, 'block(s)'), (False, 'NOTHING DECLARED')):
        path = _write(_intent_doc(pcb, blocks=blocks))
        out = os.path.join(tempfile.mkdtemp(), 'o.png')
        try:
            r = subprocess.run(
                [sys.executable, '-X', 'utf8', tool, BOARD,
                 '--intent', path, '--size', '260', '-o', out],
                capture_output=True, text=True, encoding='utf-8',
                errors='replace', env=env, cwd=ROOT, timeout=240)
        finally:
            os.unlink(path)
        blob = (r.stdout or '') + (r.stderr or '')
        if 'Traceback' in blob:
            fail('render_placement --intent raised: %s' % blob[-300:])
            continue
        if want not in blob:
            fail('with blocks=%s the run never said %r: %s'
                 % (blocks, want, blob[-300:]))
        else:
            print('    blocks=%-5s -> %r on the console' % (blocks, want))
    if len(_FAIL) == _mark:
        print('  PASS: the still renderer says what it drew, both ways')


TESTS = (
    test_a_declared_rect_lands_where_it_was_declared,
    test_the_keepout_hatch_stays_inside_its_own_box,
    test_an_empty_intent_says_it_drew_nothing,
    test_the_verdict_recolours_held_and_drifted,
    test_render_placement_announces_the_plan_either_way,
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
