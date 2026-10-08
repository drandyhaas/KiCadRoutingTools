#!/usr/bin/env python3
"""The stage3d 3D board renders, deterministically, on SwiftShader (#1081).

Self-skips (exit 77) when this machine cannot render it at all -- no Node, no
`npm ci` in `py_router/stage3d`, no Chromium -- and SAYS which, because a skip
that does not name its cause reads like a pass.

What it pins, on the film_chain fixture (a flip, a glide, copper):

  * **the renderer is SwiftShader**, named in the result, so a GPU render --
    whose pixels depend on the machine -- is refused rather than trusted;
  * **two renders of the same timeline are the same bytes**, state for
    state: the page reads no clock, and the renderer is a CPU rasteriser;
  * **the pictures carry the story**: a state with copper differs from one
    without, the mid-flip state differs from both faces, and the back-side
    state is not the front;
  * **one frame per distinct state**, and the time per state is printed so a
    regression in cost is visible.
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

OK, WHY = R3.available()
if not OK:
    print('SKIP: the 3D board cannot render on this machine: %s' % WHY)
    sys.exit(77)

from PIL import Image, ImageChops                               # noqa: E402
import film_chain_1081 as FC                                    # noqa: E402
from kicad_parser import parse_kicad_pcb                        # noqa: E402
from stage3d import scene as SC                                 # noqa: E402
from stage3d import timeline as TL                              # noqa: E402

_FAIL = []


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


def test_the_board_renders_deterministically():
    tmp = tempfile.mkdtemp(prefix='t1081r_')
    try:
        with FC.Chain() as c:
            out = {}
            tr = FC.rip_trace(c.boards[-1], os.path.join(c.dir, 'tr.json'))
            _f, _m, st, _g = FC.film(c.boards, size=480,
                                     stage_out=out, traces={3: tr})
            tl = TL.build(out)
            sc = SC.build_scene(parse_kicad_pcb(c.boards[-1]))
            W, H = 320, 240
            a, ia, wa = R3.render(sc, tl, width=W, height=H,
                                  out_dir=os.path.join(tmp, 'a'))
            b, ib, wb = R3.render(sc, tl, width=W, height=H,
                                  out_dir=os.path.join(tmp, 'b'))
            print('    %s' % wa)
            print('    %s' % ia.get('tools'))
            _check(a is not None and b is not None,
                   'both renders produced frames (%s / %s)' % (wa, wb))
            if a is None or b is None:
                return
            _check('swiftshader' in str(ia.get('renderer')).lower(),
                   'the renderer is SwiftShader (%r)' % ia.get('renderer'))
            _check(len(a) == len(tl['states']),
                   'one frame per distinct state (%d)' % len(a))
            same = sum(1 for x, y in zip(a, b) if _sha(x) == _sha(y))
            _check(same == len(a), 'two renders are byte-identical, state '
                   'for state (%d of %d)' % (same, len(a)))
            with Image.open(a[0]) as im:
                _check(im.size == (W, H), 'frames are the board box size '
                       '(%s)' % (im.size,))
            flip_a, flip_b = [(x, y) for k, x, y in st.frame_log()
                              if k == 'flip'][0]
            fr = tl['frames']
            front = fr[flip_a - 1]
            mid = fr[(flip_a + flip_b) // 2]
            back = fr[flip_b + 1]
            last = fr[-1]
            _check(_diff(a[front], a[mid]) > 500 and
                   _diff(a[mid], a[back]) > 500,
                   'the mid-flip frame differs from both faces')
            _check(_diff(a[front], a[back]) > 500,
                   'the back is not the front')
            _check(_diff(a[back], a[last]) > 50,
                   'the copper the film reveals changes the picture')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_bodies_sit_on_their_own_face():
    """A front part's body stands on the TOP face, a back part's hangs below
    the BOTTOM one -- measured in the scene graph. A back body used to sit
    inside the board, and a 3 mm back-side connector poked out of the top
    (the phase-6 verifier: 83 of 83 back parts on orangecrab, 4 of 4 here)."""
    # a TRACKED board with back-side parts (test_457: a fresh clone has
    # nothing else)
    board = os.path.join(ROOT, 'kicad_files',
                         'rp2350_fpga_eensy_prePlane.kicad_pcb')
    pcb = parse_kicad_pcb(board)
    sc = SC.build_scene(pcb)
    layers = list(pcb.board_info.copper_layers)
    pose = {ref: [fp.x, fp.y, fp.rotation or 0.0, fp.layer or 'F.Cu']
            for ref, fp in pcb.footprints.items()}
    tl = {'layers': layers, 'segs': [], 'vias': [], 'epochs': [pose],
          'frames': [0], 'states': [{'ns': 0, 'nv': 0, 'hide': [],
                                     'hl_s': [], 'hl_v': [], 'color': None,
                                     'epoch': 0, 'moving': {}, 'angle': 0.0,
                                     'active': None}],
          'side_rule': 'stage'}
    tmp = tempfile.mkdtemp(prefix='t1081p_')
    try:
        pngs, info, why = R3.render(sc, tl, width=240, height=160,
                                    out_dir=tmp, probe=0)
        pr = info.get('probe') or {}
        bodies, d = pr.get('bodies') or {}, pr.get('thickness', 1.6)
        back = [r for r, p in sc['parts'].items()
                if p['side'] == 'B' and r in bodies]
        front = [r for r, p in sc['parts'].items()
                 if p['side'] == 'F' and r in bodies]
        _check(pngs is not None and back and front,
               '%s: %d front and %d back bodies probed (%s)'
               % (os.path.basename(board), len(front), len(back), why))
        bad_b = [r for r in back if bodies[r][1] > 1e-6]
        bad_f = [r for r in front if bodies[r][0] < d - 1e-6]
        _check(not bad_b, 'every back body hangs below the bottom face '
               '(wrong: %s)' % bad_b[:5])
        _check(not bad_f, 'every front body stands on the top face '
               '(wrong: %s)' % bad_f[:5])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _one_state_timeline(pcb, zones=()):
    layers = list(pcb.board_info.copper_layers)
    pose = {ref: [fp.x, fp.y, fp.rotation or 0.0, fp.layer or 'F.Cu']
            for ref, fp in pcb.footprints.items()}
    st = {'ns': 0, 'nv': 0, 'hide': [], 'hl_s': [], 'hl_v': [],
          'color': None, 'epoch': 0, 'moving': {}, 'angle': 0.0,
          'active': None, 'zones': sorted(zones)}
    return {'layers': layers, 'segs': [], 'vias': [], 'epochs': [pose],
            'frames': [0], 'states': [st], 'side_rule': 'test'}


def _probe(board, zones=()):
    pcb = parse_kicad_pcb(board)
    sc = SC.build_scene(pcb)
    tmp = tempfile.mkdtemp(prefix='t1081z_')
    try:
        pngs, info, why = R3.render(sc, _one_state_timeline(pcb, zones),
                                    width=240, height=160, out_dir=tmp,
                                    probe=0)
        return pcb, sc, info.get('probe') or {}, why
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_pours_pads_and_drills_are_on_the_3d_board():
    """#1090. The plane pours show from the frame their net is revealed (as
    the 2D film draws them); a custom pad is its REAL outline, not the
    parser's board-space bbox; and every drill is drawn, at its own centre."""
    board = os.path.join(ROOT, 'kicad_files',
                         'lvds_converter_dualclk_gnd.kicad_pcb')
    pcb, sc, pr, why = _probe(board)
    nets = sorted({z['net'] for z in sc['pours']})
    _check(sc['pours'] and pr.get('pours_total') == len(sc['pours'])
           and pr.get('pours') == 0,
           '%d pours built, none shown before their net is revealed (%s)'
           % (len(sc['pours']), why))
    _pcb, _sc, pr2, _w = _probe(board, zones=nets[:1])
    want = sum(1 for z in sc['pours'] if z['net'] == nets[0])
    _check(pr2.get('pours') == want,
           'revealing net %s shows exactly its %d pour(s) (%s)'
           % (nets[0], want, pr2.get('pours')))
    board = os.path.join(ROOT, 'kicad_files',
                         'rp2350_fpga_eensy_prePlane.kicad_pcb')
    pcb, sc, pr, why = _probe(board)
    customs = sum(1 for p in sc['parts'].values() for q in p['pads']
                  if q[9])
    drills = sum(1 for p in sc['parts'].values() for q in p['pads']
                 if q[7] > 0)
    offset = sum(1 for p in sc['parts'].values() for q in p['pads']
                 if q[8])
    _check(customs >= 16 and pr.get('custom', 0) >= customs,
           '%d custom pads drawn as their outlines (%s)'
           % (customs, pr.get('custom')))
    _check(drills and offset and pr.get('holes', 0) >= drills,
           '%d drills drawn (%d at an offset centre) (%s)'
           % (drills, offset, pr.get('holes')))
    # the offset drill really is off the copper's centre, in the part frame
    fp = pcb.footprints['U8']
    pad = next(q for q in fp.pads if getattr(q, 'hole_x', None) is not None)
    loc = next(q for q in sc['parts']['U8']['pads'] if q[8])
    gx, gy = SC._to_local(fp, pad.hole_x, pad.hole_y)
    _check(abs(loc[8][0] - gx) < 1e-4 and abs(loc[8][1] - gy) < 1e-4
           and abs(loc[8][0] - loc[0]) + abs(loc[8][1] - loc[1]) > 0.05,
           'U8\'s drill sits at its own centre, not the copper\'s')


def _span(png):
    """The drawn extent's larger fraction of the frame (width or height):
    pixels that differ from the corner's ground."""
    from PIL import Image
    with Image.open(png) as im:
        im = im.convert('RGB')
        w, h = im.size
        g = im.getpixel((1, 1))
        hit = [(x, y) for x in range(0, w, 2) for y in range(0, h, 2)
               if sum(abs(a - b) for a, b in zip(im.getpixel((x, y)), g)) > 45]
    if not hit:
        return 0.0
    xs, ys = [q[0] for q in hit], [q[1] for q in hit]
    return max((max(xs) - min(xs)) / float(w), (max(ys) - min(ys)) / float(h))


def test_a_pour_outline_off_the_board_does_not_move_the_camera():
    """The camera is fitted per state, and a pour is built from the zone
    OUTLINE, which may run past the board (KiCad clips the fill). lvds has a
    net-0 B.Cu zone at (0,0), 80 mm from the board: counted in the fit, its
    reveal shrank the board to a quarter of the box (the review of the
    per-state fit)."""
    board = os.path.join(ROOT, 'kicad_files',
                         'lvds_converter_dualclk_gnd.kicad_pcb')
    pcb = parse_kicad_pcb(board)
    sc = SC.build_scene(pcb)
    tl = _one_state_timeline(pcb)
    st = dict(tl['states'][0], zones=sorted({z['net'] for z in sc['pours']}))
    tl['states'].append(st)
    tl['frames'] = [0, 1]
    tmp = tempfile.mkdtemp(prefix='t1081f_')
    try:
        pngs, _info, why = R3.render(sc, tl, width=320, height=200,
                                     out_dir=tmp)
        a, b = (_span(pngs[0]), _span(pngs[1])) if pngs else (0, 0)
        _check(pngs and a > 0.7 and b > 0.9 * a,
               'the board spans %.0f %% of the frame, and %.0f %% with every '
               'pour revealed (%s)' % (100 * a, 100 * b, why))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


TESTS = (test_the_board_renders_deterministically,
         test_bodies_sit_on_their_own_face,
         test_pours_pads_and_drills_are_on_the_3d_board,
         test_a_pour_outline_off_the_board_does_not_move_the_camera)


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
