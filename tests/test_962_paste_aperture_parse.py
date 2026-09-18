#!/usr/bin/env python3
"""#962: the paste stencil reaches PCBData, and the graphic-copper fields it relies on.

Before #962 nothing modelled a solder-paste opening. On esp_prog U2 (SOT-89):
- the land tab is net-0 F.Cu graphic copper;
- the stencil opening is an F.Paste `fp_poly`;
- the declared pad 2 is on F.Cu ONLY.

The one object that says where solder goes was dropped by the parser, so the
router placed a via inside it and nothing could see it.

Invariants gated here, on the TEXT parse path. The pcbnew path and pcbnew's
own resolved margin are gated by tests/gui_parity/test_962_paste_parity.py.

**On the corpus (esp_prog, glasgow):**
1. esp_prog U2 has exactly ONE graphic opening: the F.Paste poly. The F.Cu tab
   and the F.Mask poly are not openings, and pad 2 (F.Cu only) opens nothing.
2. Its geometry is the 4.5 x 1.6 mm poly `local_to_global` puts at U2's pose,
   plus half the 0.1 stroke.
3. It concerns `Net-(C1-Pad1)` vias only, not GND or +3.3V. A via at the
   position the router used lands inside it.
4. Pads 1/3 take the board margin (-0.0508). A pad with its own
   `(solder_paste_margin 0.127)` takes its own.
5. glasgow J1's pin-in-paste openings concern the through-hole pad's net.

**On synthetic boards:**

6. Precedence, per axis (pad, then footprint, then board, margin and ratio
   resolved separately):
   - the clamp at -size/2;
   - both ratio spellings;
   - `*.Paste`;
   - B side;
   - a 45-degree footprint;
   - a paste-only pad gets NO margin.
7. Graphic shapes:
   - rect, circle and line openings;
   - an unfilled poly is a band, not an area;
   - a board-level `gr_poly` on F.Paste;
   - a copper poly is not an opening.

**Graphic copper:**

8. `drawn_width` is the stroke AS DRAWN. watchy AE1's filled polys are drawn at
   0 and modelled at TRACK_WIDTH. `graphic_kind` and `graphic_circle` are
   recorded. A copper `fp_rect` at a non-cardinal angle is kind `poly`,
   because pcbnew converts it.

**Section 9 pins what the Phase-1 verifier refuted:**
- B1: QFN windowpanes concern the net of the EP around them (watchy U4, and
  a synthetic EP).
- B2: KiCad's fill rule when there is no fill token.
- S1: a custom pad's ratio is sized from its ANCHOR.
- S2: an explicit 0 override is unset.
- S4: the net index cannot survive an in-place aperture change.
- S5: the own-pad-lift arm, with an opening over a pad-touching tab only.
- S6: a strip crossing a pad.

Run:
    python3 tests/test_962_paste_aperture_parse.py
"""

import math
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))

from kicad_parser import parse_kicad_pcb, local_to_global  # noqa: E402
import paste_apertures as pa  # noqa: E402
import routing_defaults as defaults  # noqa: E402

RUN_ALL_FAST_OK = True

FAILS = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _net(p, name):
    return next(nid for nid, n in p.nets.items() if n.name == name)


def _parse_text(text):
    fd, path = tempfile.mkstemp(suffix='.kicad_pcb')
    with os.fdopen(fd, 'w', encoding='utf-8') as fh:
        fh.write(text)
    try:
        return parse_kicad_pcb(path)
    finally:
        os.unlink(path)


def _board(body, setup=''):
    return ('(kicad_pcb (version 20240108) (generator "t")\n'
            ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (35 "F.Paste" user)'
            ' (34 "B.Paste" user) (44 "Edge.Cuts" user))\n'
            ' (setup %s)\n'
            ' (net 0 "")\n (net 1 "/A")\n (net 2 "/B")\n%s\n)' % (setup, body))


def _pad(num, extra='', layers='"F.Cu" "F.Paste"', at='0 0', size='1 0.5',
         net='(net 1 "/A")', kind='smd rect'):
    return ('   (pad "%s" %s (at %s) (size %s) (layers %s) %s %s)'
            % (num, kind, at, size, layers, net, extra))


def _fp(inner, at='10 20', header='', ref='U1', layer='F.Cu'):
    return ('(footprint "L:P" (layer "%s") (at %s)\n'
            '   (property "Reference" "%s")\n%s\n%s)' % (layer, at, ref, header, inner))


def _approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


def main():
    # ---------------- corpus: esp_prog U2 -----------------------------------
    esp = parse_kicad_pcb(os.path.join(ROOT_DIR, 'kicad_files', 'esp_prog.kicad_pcb'))
    u2 = [a for a in esp.paste_apertures if a.owner_ref == 'U2']
    g = [a for a in u2 if a.source == 'graphic']
    check('1. U2 has exactly one graphic paste opening', len(g) == 1, str(g))
    check('1. the tab (F.Cu) and F.Mask polys are NOT openings',
          all(a.layer == 'F.Paste' for a in u2))
    check('1. pad 2 (F.Cu only) opens nothing',
          not any(a.pad_number == '2' for a in u2),
          str([a.label() for a in u2]))
    if g:
        ap = g[0]
        fp = esp.footprints['U2']
        corners = [local_to_global(fp.x, fp.y, fp.rotation, x, y)
                   for x, y in ((-0.8, -2.3), (0.8, 2.2))]
        xs = sorted(c[0] for c in corners)
        ys = sorted(c[1] for c in corners)
        want = (xs[0] - 0.05, ys[0] - 0.05, xs[1] + 0.05, ys[1] + 0.05)
        check('2. its bounds are the posed 4.5x1.6 poly plus half the 0.1 stroke',
              all(_approx(a, b, 1e-6) for a, b in zip(ap.bounds, want)),
              f'{ap.bounds} vs {want}')
        check('2. it is filled, with width 0.1', ap.filled and _approx(ap.width, 0.1))
        c1 = _net(esp, 'Net-(C1-Pad1)')
        check('3. it concerns Net-(C1-Pad1) vias',
              ap in pa.apertures_for_net(esp, c1))
        others = {n for n in (_net(esp, 'GND'), _net(esp, '/+3.3V'))
                  if ap in pa.apertures_for_net(esp, n)}
        check('3. ... and NOT GND or +3.3V (pads 1/3 have their own openings)',
              not others, str(others))
        # the via the router placed in the reproduction, re-posed onto this
        # board: centre of the opening's long axis
        cx = (ap.bounds[0] + ap.bounds[2]) / 2
        cy = (ap.bounds[1] + ap.bounds[3]) / 2
        check('3. a via at the opening centre penetrates it by its radius',
              _approx(pa.via_paste_penetration(cx, cy, 0.5, ap), 0.25))
        check('3. a via 1 mm clear of the opening does not touch it',
              pa.via_paste_penetration(ap.bounds[2] + 1.0 + 0.25, cy, 0.5, ap) < 0)
    p13 = [a for a in u2 if a.pad_number in ('1', '3')]
    check('4. pads 1/3 take the board margin -0.0508 per axis',
          len(p13) == 2 and all(_approx(m, -0.0508) for a in p13 for m in a.margin),
          str([(a.pad_number, a.margin) for a in p13]))
    own = [a for a in esp.paste_apertures
           if a.source == 'pad' and _approx(a.margin[0], 0.127)]
    check('4. a pad with its own (solder_paste_margin 0.127) keeps it',
          len(own) >= 2, str([a.label() for a in own][:4]))

    # ---------------- corpus: glasgow J1 pin-in-paste --------------------------
    gl = parse_kicad_pcb(os.path.join(ROOT_DIR, 'kicad_files', 'glasgow_revC.kicad_pcb'))
    j1 = [a for a in gl.paste_apertures if a.owner_ref == 'J1' and a.source == 'graphic']
    j1_nets = set()
    for a in j1:
        j1_nets |= set(pa.aperture_nets(gl, a))
    th = {p.net_id for p in gl.footprints['J1'].pads
          if p.pad_type == 'thru_hole' and p.net_id}
    check('5. glasgow J1 has pin-in-paste graphic openings', len(j1) >= 4, str(len(j1)))
    check('5. ... that concern the through-hole pads\' net',
          bool(j1_nets) and j1_nets <= th, f'{j1_nets} vs TH {th}')

    # ---------------- synthetic: precedence -------------------------------------
    setup = '(pad_to_paste_clearance 0.1) (pad_to_paste_clearance_ratio -0.1)'
    body = _fp('\n'.join([
        _pad('A', '(solder_paste_margin 0.02)'),                  # own margin
        _pad('B', ''),                                            # inherit fp
        _pad('C', '(solder_paste_margin_ratio -0.9)'),            # clamp
        _pad('D', '', layers='"F.Cu" "*.Paste"'),                 # *.Paste
        _pad('E', '', layers='"B.Cu" "B.Paste"'),                 # B side
        _pad('F', '', layers='"F.Paste"', net=''),                # paste-only
    ]), header='(solder_paste_margin 0.05)')
    body += '\n' + _fp(_pad('G', '', size='1 0.5'), at='30 20',
                       header='(solder_paste_ratio -0.2)', ref='U2')
    body += '\n' + _fp(_pad('H', '', size='1 0.5'), at='50 20',
                       header='(solder_paste_margin_ratio -0.2)', ref='U3')
    body += '\n' + _fp(_pad('K', '', size='1 0.5'), at='70 20 45', ref='U4')
    p = _parse_text(_board(body, setup))
    by = {(a.owner_ref, a.pad_number, a.layer): a for a in p.paste_apertures}
    A = by.get(('U1', 'A', 'F.Paste'))
    check('6. pad margin beats footprint; the ratio still comes from the board',
          A is not None and _approx(A.margin[0], 0.02 + 1.0 * -0.1)
          and _approx(A.margin[1], 0.02 + 0.5 * -0.1), str(A and A.margin))
    B = by.get(('U1', 'B', 'F.Paste'))
    check('6. a pad with no override takes the footprint margin + board ratio',
          B is not None and _approx(B.margin[0], 0.05 - 0.1)
          and _approx(B.margin[1], 0.05 - 0.05), str(B and B.margin))
    # A margin at or below -size/2 CLOSES the opening. KiCad clamps it at
    # -size/2, and a zero-width opening is no opening, so the result is None,
    # never a negative size.
    C = by.get(('U1', 'C', 'F.Paste'))
    check('6. a ratio that closes the opening yields NO opening',
          C is None, str(C and C.margin))
    check('6. *.Paste opens F.Paste and B.Paste',
          ('U1', 'D', 'F.Paste') in by and ('U1', 'D', 'B.Paste') in by)
    check('6. a B-side pad opens B.Paste only',
          ('U1', 'E', 'B.Paste') in by and ('U1', 'E', 'F.Paste') not in by)
    F = by.get(('U1', '', 'F.Paste'))     # a paste-only opening carries no number
    check('6. a paste-only pad is an opening with NO margin (KiCad rule)',
          F is not None and F.source == 'paste_only_pad' and F.margin == (0.0, 0.0),
          str(F and (F.source, F.margin)))
    G = by.get(('U2', 'G', 'F.Paste'))
    H = by.get(('U3', 'H', 'F.Paste'))
    check('6. footprint (solder_paste_ratio R) is read',
          G is not None and _approx(G.margin[0], 0.1 - 0.2), str(G and G.margin))
    check('6. footprint (solder_paste_margin_ratio R) is read too',
          H is not None and _approx(H.margin[0], 0.1 - 0.2), str(H and H.margin))
    K = by.get(('U4', 'K', 'F.Paste'))
    if K is not None:
        pad_k = next(q for q in p.footprints['U4'].pads)
        # the opening follows the rotated copper: a point on the pad's own
        # long axis 0.55 mm out is inside (0.5 + margin 0.1 - 0.1*1 = 0.5?),
        # measured against the inflated pad through check_drc's own geometry
        from check_drc import point_to_pad_distance
        inside = pa.aperture_distance(pad_k.global_x, pad_k.global_y, K)
        check('6. a 45-degree pad opening is centred on the rotated pad',
              _approx(inside, 0.0) and K.shape_pad.rect_rotation == pad_k.rect_rotation,
              f'{inside} rr={K.shape_pad.rect_rotation}/{pad_k.rect_rotation}')
        check('6. the 45-degree opening agrees with point_to_pad_distance',
              _approx(pa.aperture_distance(pad_k.global_x + 2, pad_k.global_y, K),
                      point_to_pad_distance(pad_k.global_x + 2, pad_k.global_y, K.shape_pad)))
    else:
        check('6. a 45-degree footprint pad has an opening', False)

    # ---------------- synthetic: graphic shapes --------------------------------
    shapes = '\n'.join([
        '   (fp_rect (start 0 0) (end 1 1) (stroke (width 0.1) (type solid)) '
        '(fill yes) (layer "F.Paste") (uuid "r"))',
        '   (fp_circle (center 5 0) (end 5.5 0) (stroke (width 0) (type solid)) '
        '(fill yes) (layer "F.Paste") (uuid "c"))',
        '   (fp_line (start 10 0) (end 12 0) (stroke (width 0.2) (type solid)) '
        '(layer "F.Paste") (uuid "l"))',
        '   (fp_poly (pts (xy 20 0) (xy 22 0) (xy 22 2) (xy 20 2)) '
        '(stroke (width 0.2) (type solid)) (fill no) (layer "F.Paste") (uuid "u"))',
        '   (fp_poly (pts (xy 30 0) (xy 31 0) (xy 31 1)) '
        '(stroke (width 0.1) (type solid)) (fill yes) (layer "F.Cu") (uuid "cu"))',
    ])
    body = _fp(shapes + '\n' + _pad('1', layers='"F.Cu"'), at='0 0')
    body += ('\n (gr_poly (pts (xy 100 100) (xy 101 100) (xy 101 101)) '
             '(stroke (width 0) (type solid)) (fill yes) (layer "F.Paste") (uuid "g"))')
    p = _parse_text(_board(body))
    gr = {a.uuid: a for a in p.paste_apertures if a.source == 'graphic'}
    check('7. rect, circle, line, unfilled poly and board-level poly are openings; '
          'the copper poly is not',
          set(gr) == {'r', 'c', 'l', 'u', 'g'}, str(sorted(gr)))
    if 'c' in gr:
        check('7. a filled circle keeps its true geometry',
              gr['c'].circle == (5.0, 0.0, 0.5)
              and _approx(pa.aperture_distance(5.0, 0.0, gr['c']), 0.0)
              and _approx(pa.aperture_distance(6.0, 0.0, gr['c']), 0.5))
    if 'l' in gr:
        check('7. a stroked line is an open band of half its width',
              _approx(pa.aperture_distance(11.0, 0.1, gr['l']), 0.0)
              and _approx(pa.aperture_distance(11.0, 0.3, gr['l']), 0.2))
    if 'u' in gr:
        check('7. an UNFILLED poly is a band, not an area (its centre is clear)',
              not gr['u'].filled
              and _approx(pa.aperture_distance(21.0, 1.0, gr['u']), 1.0 - 0.1))
    if 'g' in gr:
        check('7. a board-level gr_poly on F.Paste is an owner-less opening',
              gr['g'].owner_ref == '')

    # ---------------- graphic copper fields --------------------------------
    w = parse_kicad_pcb(os.path.join(ROOT_DIR, 'kicad_files', 'watchy.kicad_pcb'))
    ae1 = [s for s in w.segments if s.graphic and s.owner_ref == 'AE1']
    check('8. watchy AE1 copper is drawn at stroke 0 and modelled at TRACK_WIDTH',
          bool(ae1) and all(s.drawn_width == 0.0 and s.width == defaults.TRACK_WIDTH
                            for s in ae1),
          str(sorted({(s.drawn_width, s.width) for s in ae1})))
    esp_tab = [s for s in esp.segments if s.graphic and s.owner_ref == 'U2']
    check('8. esp_prog U2 tab is drawn at 0.1, kind poly',
          bool(esp_tab) and all(s.drawn_width == 0.1 and s.graphic_kind == 'poly'
                                for s in esp_tab))
    circ = ('   (fp_circle (center 1 0) (end 2 0) (stroke (width 0.2) (type solid)) '
            '(fill no) (layer "F.Cu") (uuid "cc"))')
    p = _parse_text(_board(_fp(circ + '\n' + _pad('1', layers='"F.Cu"'), at='10 20 90')))
    cs = [s for s in p.segments if s.graphic and s.graphic_kind == 'circle']
    want_c = local_to_global(10, 20, 90, 1, 0)
    check('8. a copper circle records its TRUE global centre and radius',
          bool(cs) and all(s.graphic_circle is not None
                           and _approx(s.graphic_circle[0], want_c[0])
                           and _approx(s.graphic_circle[1], want_c[1])
                           and _approx(s.graphic_circle[2], 1.0) for s in cs),
          str(cs[:1] and cs[0].graphic_circle))
    tracks = [s for s in esp.segments if not s.graphic]
    check('8. tracks carry no drawn_width / kind',
          all(s.drawn_width is None and s.graphic_kind == '' for s in tracks))
    rect45 = ('   (fp_rect (start 0 0) (end 1 1) (stroke (width 0.1) (type solid)) '
              '(fill yes) (layer "F.Cu") (uuid "r45"))')
    p = _parse_text(_board(_fp(rect45 + '\n' + _pad('1', layers='"F.Cu"'), at='10 20 45')))
    check('8. a copper fp_rect in a 45-degree footprint is kind poly (as pcbnew reads it)',
          {s.graphic_kind for s in p.segments if s.graphic} == {'poly'})

    # ---------------- 9: the phase-1 verification findings ----------------------
    # B1: QFN windowpanes. Paste-only panes sit INSIDE a copper exposed pad that
    # opens no paste of its own. The EP's centre and perimeter can miss every
    # pane, so association must also test the pane on the pad.
    wat = parse_kicad_pcb(os.path.join(ROOT_DIR, 'kicad_files', 'watchy.kicad_pcb'))
    panes = [a for a in wat.paste_apertures if a.owner_ref == 'U4'
             and a.source == 'paste_only_pad']
    gnd_w = _net(wat, 'GND')
    check('9/B1. watchy U4 has paste-only windowpanes', len(panes) >= 16, str(len(panes)))
    check('9/B1. ... and EVERY pane concerns GND (the EP net)',
          bool(panes) and all(pa.aperture_nets(wat, a) == frozenset({gnd_w}) for a in panes),
          str([sorted(pa.aperture_nets(wat, a)) for a in panes][:4]))
    ep = ('   (pad "EP" smd rect (at 0 0) (size 4 4) (layers "F.Cu" "F.Mask") (net 1 "/A"))')
    pane_pads = '\n'.join(
        '   (pad "" smd rect (at %s) (size 0.8 0.8) (layers "F.Paste"))' % at
        for at in ('-1 -1', '1 -1', '-1 1', '1 1'))
    p = _parse_text(_board(_fp(ep + '\n' + pane_pads, at='50 50')))
    sp = [a for a in p.paste_apertures if a.source == 'paste_only_pad']
    check('9/B1. synthetic: 4 panes inside a copper EP each concern the EP net',
          len(sp) == 4 and all(pa.aperture_nets(p, a) == frozenset({1}) for a in sp),
          str([sorted(pa.aperture_nets(p, a)) for a in sp]))
    check('9. a paste-only opening carries no pad number (pcbnew blanks it)',
          all(a.pad_number == '' for a in sp))

    # B2: no fill token. pcbnew 10 reads a token-less poly as FILLED, and a
    # token-less rect/circle as filled only at stroke width 0.
    nofill = '\n'.join([
        '   (fp_poly (pts (xy 0 0) (xy 1 0) (xy 1 1)) (stroke (width 0.1) (type solid)) '
        '(layer "F.Paste") (uuid "p"))',
        '   (fp_rect (start 3 0) (end 4 1) (stroke (width 0) (type solid)) '
        '(layer "F.Paste") (uuid "r0"))',
        '   (fp_rect (start 6 0) (end 7 1) (stroke (width 0.1) (type solid)) '
        '(layer "F.Paste") (uuid "r1"))',
        '   (fp_circle (center 10 0) (end 10.5 0) (stroke (width 0) (type solid)) '
        '(layer "F.Paste") (uuid "c0"))',
    ])
    p = _parse_text(_board(_fp(nofill + '\n' + _pad('1', layers='"F.Cu"'), at='0 0')))
    gg = {a.uuid: a for a in p.paste_apertures if a.source == 'graphic'}
    check('9/B2. no fill token: poly filled, rect@0 filled, rect@0.1 a band, circle@0 filled',
          set(gg) == {'p', 'r0', 'r1', 'c0'} and gg['p'].filled and gg['r0'].filled
          and not gg['r1'].filled and gg['c0'].filled,
          str({k: v.filled for k, v in gg.items()}))

    # S1: a custom pad's ratio is sized from its ANCHOR, not the primitive
    # extent. Measured against pcbnew: anchor 0.5, primitive 2x1, margin
    # -0.05, ratio -0.1 -> (-0.10, -0.10).
    cust = ('   (pad "1" smd custom (at 0 0) (size 0.5 0.5) (layers "F.Cu" "F.Paste") '
            '(net 1 "/A") (solder_paste_margin -0.05) (solder_paste_margin_ratio -0.1)\n'
            '     (options (clearance outline) (anchor rect))\n'
            '     (primitives (gr_poly (pts (xy -1 -0.5) (xy 1 -0.5) (xy 1 0.5) (xy -1 0.5)) '
            '(width 0) (fill yes))))')
    p = _parse_text(_board(_fp(cust, at='5 5')))
    ca = [a for a in p.paste_apertures if a.source == 'pad']
    check('9/S1. a custom pad\'s paste ratio uses the ANCHOR size (-0.10, -0.10)',
          len(ca) == 1 and all(_approx(m, -0.10) for m in ca[0].margin),
          str(ca and ca[0].margin))

    # S2: an explicit 0 is UNSET to KiCad, so the pad inherits the footprint.
    zero = _pad('Z', '(solder_paste_margin 0) (solder_paste_margin_ratio 0)')
    p = _parse_text(_board(_fp(zero, header='(solder_paste_margin 0.05)'), ''))
    za = [a for a in p.paste_apertures if a.pad_number == 'Z']
    check('9/S2. (solder_paste_margin 0) reads as unset and inherits the footprint 0.05',
          len(za) == 1 and _approx(za[0].margin[0], 0.05), str(za and za[0].margin))

    # S5: the own-pad-lift arm. The opening covers only the TAB, which touches
    # pad 1, and does not overlap the pad. Only the lift can say it concerns /A.
    tab = ('   (fp_poly (pts (xy 0.5 -0.5) (xy 3 -0.5) (xy 3 0.5) (xy 0.5 0.5)) '
           '(stroke (width 0.1) (type solid)) (fill yes) (layer "F.Cu") (uuid "t"))\n'
           '   (fp_poly (pts (xy 1.5 -0.3) (xy 2.8 -0.3) (xy 2.8 0.3) (xy 1.5 0.3)) '
           '(stroke (width 0) (type solid)) (fill yes) (layer "F.Paste") (uuid "tp"))')
    p = _parse_text(_board(_fp(_pad('1', layers='"F.Cu"', size='1 1') + '\n' + tab,
                               at='20 20')))
    tpa = [a for a in p.paste_apertures if a.uuid == 'tp']
    check('9/S5. an opening over a pad-touching TAB (not the pad) concerns the pad net',
          len(tpa) == 1 and pa.aperture_nets(p, tpa[0]) == frozenset({1}),
          str(tpa and sorted(pa.aperture_nets(p, tpa[0]))))

    # S6: a thin strip CROSSING a pad has no vertex inside the pad and the pad
    # has none inside it; dense sampling must still see the overlap.
    strip = ('   (fp_poly (pts (xy -3 0.05) (xy 3 0.05) (xy 3 0.15) (xy -3 0.15)) '
             '(stroke (width 0) (type solid)) (fill yes) (layer "F.Paste") (uuid "s"))')
    p = _parse_text(_board(_fp(_pad('1', layers='"F.Cu"', size='2 2') + '\n' + strip,
                               at='30 30')))
    st = [a for a in p.paste_apertures if a.uuid == 's']
    check('9/S6. a strip crossing a pad concerns the pad net',
          len(st) == 1 and pa.aperture_nets(p, st[0]) == frozenset({1}),
          str(st and sorted(pa.aperture_nets(p, st[0]))))

    # S4: the net index must not survive a change to the aperture list, even
    # an in-place replacement at unchanged length.
    idx1 = pa.apertures_by_net(esp)
    old = esp.paste_apertures[0]
    esp.paste_apertures[0] = esp.paste_apertures[-1]
    idx2 = pa.apertures_by_net(esp)
    esp.paste_apertures[0] = old
    idx3 = pa.apertures_by_net(esp)
    check('9/S4. an in-place aperture replacement invalidates the net index',
          idx2 is not idx1 and idx3 is not idx2)
    check('9/S4. an unchanged board reuses the index (the memo works)',
          pa.apertures_by_net(esp) is idx3)

    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
