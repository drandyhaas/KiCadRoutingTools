#!/usr/bin/env python3
"""#908: copper drawn INSIDE a footprint must reach the copper model.

The #337 scan walks board-level `gr_*` only, so a footprint's own copper -- the
drawn tab of a SOT89/DPAK, a PCB antenna, a solder-jumper bridge -- was
invisible to every obstacle builder and to check_drc, which reads this same
parser. Measured on esp_prog before the fix: kicad-cli 10.0.0 reports "Pad 2
[Net-(C1-Pad1)] of U2 on F.Cu" shorting "Polygon [<no net>] of U2 on F.Cu",
and `check_drc.py` on the same file says NO DRC VIOLATIONS FOUND.

Invariants gated here (the `gr_*` table in test_copper_graphics_layers_plural.py
is the sibling this is modelled on, including its anti-vacuity idiom):

  1. `fp_poly` on F.Cu in a pad-bearing footprint -> graphic Segments at the
     GLOBAL coordinates `local_to_global` gives, rotation included.
  2. B.Cu is read from the SHAPE's own layer; no mirror term is applied.
  3. A pad-LESS (logo) footprint emits nothing -- the writer relocates it.
  4. Silk/mask-only `fp_poly` emits nothing.
  5. A custom pad's `(primitives (gr_poly ...))` emits nothing, even on a
     copper layer -- those coordinates are PAD-local and would land as
     phantom copper (urchin carries 136 of them).
  6. `(stroke (width 0))` on a filled poly -> the TRACK_WIDTH fallback;
     an `fp_line` with width 0 -> nothing.
  7. The plural `(layers "F.Cu" "F.Mask")` form is read, and `F&B.Cu`
     emits on both copper sides -- the same answers the `gr_*` table gives.
  8. A vertex list that already repeats its first point emits N-1 segments,
     not N with a zero-length one. Every watchy antenna poly is written that
     way, so the duplicates doubled three of its board-edge findings (12
     reported where the geometry gives 9; those nine are now published as the
     `immutable-graphic` accepted class rather than counted, but a degenerate
     segment would still double them there).
  9. `owner_ref` names the DISAMBIGUATED footprint key, so a board that
     spells one reference twice attributes copper to the right block.
 10. The real corpus: esp_prog 8, tigard 4, ulx3s 24, watchy 48 graphic
     segments; orangecrab_ext_pll, splitflap_driver and glasgow_revC zero.

Run:
    python3 tests/test_908_footprint_copper.py
"""

import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522

from kicad_parser import extract_segments, local_to_global, parse_kicad_pcb
import routing_defaults as defaults

RUN_ALL_FAST_OK = True

NAME_TO_ID = {'/A': 1, '/B': 2}


def _board(body):
    return ('(kicad_pcb (version 20221018)\n (net 0 "")\n (net 1 "/A")\n'
            ' (net 2 "/B")\n%s\n)' % body)


def _fp(inner, at='10 20 -90', pads=1, ref='U1'):
    pad_txt = '\n'.join(
        '   (pad "%d" smd rect (at %d 0) (size 0.6 0.6) (layers "F.Cu") '
        '(net 1 "/A"))' % (i + 1, i) for i in range(pads))
    return ('(footprint "L:P" (layer "F.Cu") (at %s)\n'
            '   (property "Reference" "%s")\n%s\n%s)' % (at, ref, pad_txt, inner))


def _poly(layer='F.Cu', pts='(xy 0 0) (xy 1 0) (xy 1 1) (xy 0 1)',
          stroke='(stroke (width 0) (type solid)) (fill yes)'):
    return ('   (fp_poly (pts %s)\n     %s\n     (layer "%s") (uuid "p1"))'
            % (pts, stroke, layer))


def _graphics(content):
    return [s for s in extract_segments(content, NAME_TO_ID)
            if getattr(s, 'graphic', False)]


def main():
    fails = []

    def check(name, cond, detail=''):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}"
              + (f"   [{detail}]" if detail and not cond else ''))
        if not cond:
            fails.append(name)

    # --- 1: geometry, with rotation ---------------------------------------
    segs = _graphics(_board(_fp(_poly())))
    want = [local_to_global(10, 20, -90, x, y)
            for x, y in ((0, 0), (1, 0), (1, 1), (0, 1))]
    got_pts = {(round(s.start_x, 6), round(s.start_y, 6)) for s in segs}
    check('rotated fp_poly emits its transformed outline',
          len(segs) == 4
          and got_pts == {(round(x, 6), round(y, 6)) for x, y in want},
          f'{len(segs)} segs, pts={sorted(got_pts)}')
    check('emitted copper is graphic, net 0, owned by U1',
          bool(segs) and all(s.graphic and s.net_id == 0
                             and s.owner_ref == 'U1' for s in segs))
    check('the transform is NOT the identity (rotation really applied)',
          bool(segs) and got_pts != {(0.0, 0.0), (1.0, 0.0),
                                     (1.0, 1.0), (0.0, 1.0)})

    # --- 2: B.Cu comes from the shape, not the footprint ------------------
    segs = _graphics(_board(_fp(_poly(layer='B.Cu'))))
    check('a B.Cu fp_poly lands on B.Cu',
          bool(segs) and all(s.layer == 'B.Cu' for s in segs))
    b_pts = {(round(s.start_x, 6), round(s.start_y, 6)) for s in segs}
    check('no extra mirror term is applied to a B.Cu shape',
          b_pts == got_pts, f'{sorted(b_pts)}')

    # --- 3/4/5: the three negative controls -------------------------------
    check('a pad-LESS logo footprint emits nothing',
          _graphics(_board(_fp(_poly(), pads=0))) == [])
    check('a silk-only fp_poly emits nothing',
          _graphics(_board(_fp(_poly(layer='F.SilkS')))) == [])
    prim = ('   (pad "9" smd custom (at 3 0) (size 0.4 0.4) (layers "F.Cu")\n'
            '     (net 2 "/B")\n'
            '     (primitives (gr_poly (pts (xy 0 0) (xy 9 0) (xy 9 9))\n'
            '       (stroke (width 0.1) (type solid)) (fill yes)\n'
            '       (layer "F.Cu"))))')
    check('a custom pad\'s gr_poly primitive emits nothing',
          _graphics(_board(_fp(prim))) == [])

    # --- 6: widths ---------------------------------------------------------
    segs = _graphics(_board(_fp(_poly())))
    check('a 0-stroke filled poly falls back to TRACK_WIDTH',
          bool(segs) and all(s.width == defaults.TRACK_WIDTH for s in segs),
          f'{sorted({s.width for s in segs})}')
    line0 = ('   (fp_line (start 0 0) (end 1 0)\n'
             '     (stroke (width 0) (type solid)) (layer "F.Cu") (uuid "l0"))')
    check('an fp_line with width 0 emits nothing',
          _graphics(_board(_fp(line0))) == [])
    line1 = ('   (fp_line (start 0 0) (end 1 0)\n'
             '     (stroke (width 0.2) (type solid)) (layer "F.Cu") (uuid "l1"))')
    lsegs = _graphics(_board(_fp(line1)))
    check('a stroked fp_line emits one segment at its own width',
          len(lsegs) == 1 and lsegs[0].width == 0.2)

    # --- 7: the layer-token forms -----------------------------------------
    plural = ('   (fp_poly (pts (xy 0 0) (xy 1 0) (xy 1 1))\n'
              '     (stroke (width 0.1) (type solid)) (fill yes)\n'
              '     (layers "F.Cu" "F.Mask") (uuid "p2"))')
    psegs = _graphics(_board(_fp(plural)))
    check('the plural (layers ...) form is read, mask member dropped',
          len(psegs) == 3 and all(s.layer == 'F.Cu' for s in psegs),
          f'{len(psegs)} segs, layers={sorted({s.layer for s in psegs})}')
    two = plural.replace('(layers "F.Cu" "F.Mask")', '(layer "F&B.Cu")')
    tsegs = _graphics(_board(_fp(two)))
    check('F&B.Cu emits on both copper sides',
          {s.layer for s in tsegs} == {'F.Cu', 'B.Cu'} and len(tsegs) == 6,
          f'{len(tsegs)} segs, {sorted({s.layer for s in tsegs})}')

    # --- 8: a repeated closing vertex is not a zero-length segment --------
    closed = _poly(pts='(xy 0 0) (xy 1 0) (xy 1 1) (xy 0 1) (xy 0 0)')
    csegs = _graphics(_board(_fp(closed)))
    check('a poly repeating its first point emits 4 edges, not 5',
          len(csegs) == 4, f'{len(csegs)}')
    check('no zero-length graphic segment is ever emitted',
          bool(csegs) and all((s.start_x, s.start_y) != (s.end_x, s.end_y)
                              for s in csegs))

    # --- 9: owner_ref uses the DISAMBIGUATED key --------------------------
    dup = _board(_fp(_poly(), ref='TP4') + '\n'
                 + _fp(_poly(), at='30 40 0', ref='TP4'))
    dsegs = _graphics(dup)
    check('two blocks sharing a reference get distinct owner keys',
          {s.owner_ref for s in dsegs} == {'TP4', 'TP4~2'},
          f'{sorted({s.owner_ref for s in dsegs})}')

    # --- 10: the real corpus ----------------------------------------------
    expect = {'esp_prog': 8, 'tigard': 4, 'ulx3s': 24, 'watchy': 48,
              'orangecrab_ext_pll': 0, 'splitflap_driver': 0,
              'glasgow_revC': 0}
    seen = 0
    for board, want_n in sorted(expect.items()):
        path = os.path.join(ROOT_DIR, 'kicad_files', f'{board}.kicad_pcb')
        if not os.path.isfile(path):
            check(f'{board}: corpus board present', False, path)
            continue
        pcb = parse_kicad_pcb(path)
        got = [s for s in pcb.segments if getattr(s, 'graphic', False)]
        seen += 1
        check(f'{board}: {want_n} graphic segments', len(got) == want_n,
              f'got {len(got)}')
        check(f'{board}: every graphic names an owner',
              all(s.owner_ref for s in got) if got else True)
    check('the corpus section actually ran', seen == len(expect),
          f'{seen}/{len(expect)}')

    print(f"\n{'FAILED: ' + ', '.join(fails) if fails else 'ALL PASS'}")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
