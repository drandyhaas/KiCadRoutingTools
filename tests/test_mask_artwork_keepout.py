"""Artwork on F.Mask / B.Mask is a solder-mask opening: bare copper under it
is exposed. The router must keep tracks and vias of that side's copper out of
mask artwork (board graphics and pad-less footprint graphics alike)."""
import math
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from synth import RoutingBoard  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from mask_artwork import extract_mask_artwork, _point_in  # noqa: E402

# Mask opening spanning both straight paths (y=10 and y=14), x 15..25.
BOX = (15.0, 8.0, 25.0, 16.0)


def _board(artwork):
    b = RoutingBoard(w=40, h=24)
    b.pad('A1', 5.0, 10.0, 'NET_A')
    b.pad('A2', 35.0, 10.0, 'NET_A')
    b.pad('B1', 5.0, 14.0, 'NET_B')
    b.pad('B2', 35.0, 14.0, 'NET_B')
    b.items.append(artwork)
    return b.text()


BOARD_RECT = ('  (gr_rect (start 15 8) (end 25 16) (stroke (width 0.1) (type solid)) '
              '(fill solid) (layer "F.Mask") (uuid "mask-rect"))')
# The same opening drawn inside a pad-less (logo) footprint placed at the box
# centre and rotated 90 degrees: local 8 x 10 becomes board 10 x 8.
LOGO = ('  (footprint "synth:LOGO" (layer "F.Cu") (uuid "logo-fp") (at 20 12 90)\n'
        '    (property "Reference" "G1" (at 0 0 0) (layer "F.SilkS") (uuid "logo-ref")\n'
        '      (effects (font (size 0.5 0.5) (thickness 0.1))))\n'
        '    (fp_poly (pts (xy -4 -5) (xy 4 -5) (xy 4 5) (xy -4 5)) '
        '(stroke (width 0) (type solid)) (fill solid) (layer "F.Mask") (uuid "logo-poly"))\n'
        '  )')


def _seg_hits_box(x1, y1, x2, y2, half, box):
    """Does a segment widened by `half` overlap the axis-aligned box?"""
    bx1, by1, bx2, by2 = box[0] - half, box[1] - half, box[2] + half, box[3] + half
    # Liang-Barsky clip of the centre line against the inflated box.
    t0, t1 = 0.0, 1.0
    dx, dy = x2 - x1, y2 - y1
    for p, q in ((-dx, x1 - bx1), (dx, bx2 - x1), (-dy, y1 - by1), (dy, by2 - y1)):
        if p == 0:
            if q < 0:
                return False
            continue
        t = q / p
        if p < 0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return False
    return True


def _route(text, extra=('--mask-keepouts',)):
    tmp = tempfile.mkdtemp(prefix='maskart_')
    try:
        inp = os.path.join(tmp, 'in.kicad_pcb')
        out = os.path.join(tmp, 'out.kicad_pcb')
        with open(inp, 'w', encoding='utf-8') as f:
            f.write(text)
        env = dict(os.environ, KICAD_PLANE_FINALIZE='0')
        py = os.path.join(ROOT, 'py_router')
        r = subprocess.run([sys.executable, '-X', 'utf8', os.path.join(py, 'route.py'),
                            inp, out, '--nets', 'NET_A', 'NET_B', '--escalation', 'off',
                            '--no-fix-drc-settings'] + list(extra),
                           capture_output=True, text=True, env=env, cwd=ROOT)
        assert os.path.exists(out), 'route.py wrote no output (rc %d):\n%s' % (
            r.returncode, (r.stdout + r.stderr)[-2000:])
        # the routed board passes the router's own DRC and connectivity checks
        for tool in ('check_drc.py', 'check_connected.py'):
            c = subprocess.run([sys.executable, '-X', 'utf8', os.path.join(py, tool), out,
                                '--nets', 'NET_A', 'NET_B'],
                               capture_output=True, text=True, cwd=ROOT)
            assert c.returncode == 0, '%s failed (rc %d):\n%s' % (
                tool, c.returncode, (c.stdout + c.stderr)[-2000:])
        return parse_kicad_pcb(out)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _check(text, what):
    pcb = _route(text)
    names = {n.net_id: n.name for n in pcb.nets.values()}
    routed = {names.get(s.net_id) for s in pcb.segments}
    assert {'NET_A', 'NET_B'} <= routed, '%s: nets not routed: %s' % (what, routed)
    bad = []
    for s in pcb.segments:
        if s.layer == 'F.Cu' and _seg_hits_box(s.start_x, s.start_y, s.end_x, s.end_y,
                                               s.width / 2.0, BOX):
            bad.append('track %s (%.3f,%.3f)-(%.3f,%.3f)' % (
                names.get(s.net_id), s.start_x, s.start_y, s.end_x, s.end_y))
    for v in pcb.vias:
        if _seg_hits_box(v.x, v.y, v.x, v.y, v.size / 2.0, BOX):
            bad.append('via %s (%.3f,%.3f)' % (names.get(v.net_id), v.x, v.y))
    assert not bad, '%s: F.Cu copper under F.Mask artwork: %s' % (what, '; '.join(bad))


def test_board_mask_rect_keeps_copper_out():
    _check(_board(BOARD_RECT), 'gr_rect on F.Mask')


def test_logo_footprint_mask_poly_keeps_copper_out():
    _check(_board(LOGO), 'pad-less footprint fp_poly on F.Mask')


# A mask opening around a netted pad is that pad's own opening (mask-defined
# pad): it must not wall the pad off.
PAD_OPENING = ('  (gr_rect (start 4 9) (end 6 11) (stroke (width 0.1) (type solid)) '
               '(fill solid) (layer "F.Mask") (uuid "pad-open"))')


def test_pad_own_mask_opening_stays_routable():
    pcb = _route(_board(PAD_OPENING))
    names = {n.net_id: n.name for n in pcb.nets.values()}
    routed = {names.get(s.net_id) for s in pcb.segments}
    assert 'NET_A' in routed, 'pad inside its own mask opening not routed: %s' % routed



# --- extraction contract (no routing) --------------------------------------

def _keepouts(artwork, pads=()):
    b = RoutingBoard(w=40, h=24)
    for ref, x, y, net, sx, sy in pads:
        b.pad(ref, x, y, net, sx=sx, sy=sy)
    b.items.append(artwork)
    text = b.text()
    import kicad_parser
    fps = kicad_parser.parse_kicad_pcb_text(text).footprints if hasattr(
        kicad_parser, 'parse_kicad_pcb_text') else _parse_text(text).footprints
    return extract_mask_artwork(text, fps)


def _parse_text(text):
    tmp = tempfile.mkdtemp(prefix='maskart_')
    try:
        path = os.path.join(tmp, 'b.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)
        return parse_kicad_pcb(path)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _covered(entries, pt):
    return any(_point_in(pt, e['polygon']) and not any(_point_in(pt, h) for h in e['holes'])
               for e in entries)


SMALL_RECT = ('  (gr_rect (start 0 3) (end 1 4) (stroke (width 0) (type solid)) '
              '(fill solid) (layer "F.Mask") (uuid "small"))')


def test_unrelated_long_pad_keeps_the_keepout():
    # a 10 x 0.2 mm pad at the origin ends 2.9 mm from the opening: not its owner
    assert len(_keepouts(SMALL_RECT)) == 1
    got = _keepouts(SMALL_RECT, [('L1', 0.0, 0.0, 'NET_A', 10.0, 0.2)])
    assert len(got) == 1 and _covered(got, (0.5, 3.5)), got


def test_broad_opening_touching_a_pad_keeps_the_rest():
    # opening 10 x 8, a 0.8 mm pad inside its left end: only the pad's
    # neighbourhood (OWN_MARGIN) is exempt
    rect = ('  (gr_rect (start 15 8) (end 25 16) (stroke (width 0) (type solid)) '
            '(fill solid) (layer "F.Mask") (uuid "broad"))')
    got = _keepouts(rect, [('P1', 15.5, 12.0, 'NET_A', 0.8, 0.8)])
    assert _covered(got, (20.0, 12.0)) and _covered(got, (17.0, 12.0)), got
    assert not _covered(got, (15.5, 12.0)), 'pad centre still blocked'


def test_pad_own_opening_is_exempt():
    own = ('  (gr_rect (start 4.4 9.4) (end 5.6 10.6) (stroke (width 0) (type solid)) '
           '(fill solid) (layer "F.Mask") (uuid "own"))')
    assert _keepouts(own, [('P1', 5.0, 10.0, 'NET_A', 0.8, 0.8)]) == []
    assert len(_keepouts(own)) == 1


def test_circle_is_circumscribed_and_its_hole_inscribed():
    filled = ('  (gr_circle (center 20 12) (end 30 12) (stroke (width 0) (type solid)) '
              '(fill solid) (layer "F.Mask") (uuid "c1"))')
    got = _keepouts(filled)
    a = math.pi / 32
    # 0.01 mm inside the true edge, between two vertices
    assert _covered(got, (20 + 9.99 * math.cos(a), 12 + 9.99 * math.sin(a))), got
    ring = ('  (gr_circle (center 20 12) (end 30 12) (stroke (width 1) (type solid)) '
            '(fill none) (layer "F.Mask") (uuid "c2"))')
    got = _keepouts(ring)
    for rad in (9.51, 10.49):
        assert _covered(got, (20 + rad * math.cos(a), 12 + rad * math.sin(a))), rad


def test_arc_stroke_covers_its_true_band():
    # half circle radius 10, stroke 0.2: points just inside the outer edge
    arc = ('  (gr_arc (start 10 12) (mid 20 2) (end 30 12) (stroke (width 0.2) (type solid)) '
           '(layer "F.Mask") (uuid "a1"))')
    got = _keepouts(arc)
    for k in range(1, 64):
        ang = math.pi + math.pi * k / 64.0
        pt = (20 + 10.09 * math.cos(ang), 12 + 10.09 * math.sin(ang))
        assert _covered(got, pt), (k, pt)


def test_mask_keepouts_are_opt_in():
    """Without --mask-keepouts the artwork is extracted but routing ignores it."""
    from mask_artwork import routing_keepouts
    pcb = _parse_text(_board(BOARD_RECT))
    assert pcb.board_info.mask_artwork, 'no mask artwork extracted'
    assert routing_keepouts(pcb.board_info) == list(pcb.board_info.keepouts)
    pcb.board_info.mask_keepouts = True
    assert len(routing_keepouts(pcb.board_info)) == (
        len(pcb.board_info.keepouts) + len(pcb.board_info.mask_artwork))


def test_text_is_a_bounding_box():
    """Mask text keeps out its whole (generous) bounding box, not its strokes."""
    txt = ('  (gr_text "ABCD" (at 20 12 0) (layer "F.Mask") (uuid "t1") '
           '(effects (font (size 1 1) (thickness 0.15))))')
    got = _keepouts(txt)
    assert len(got) == 1 and got[0]['layers'] == {'F.Cu'}, got
    # between glyphs and at the box ends: covered; well outside: free
    for pt in ((20.0, 12.0), (18.6, 12.0), (21.4, 12.0), (20.0, 12.45)):
        assert _covered(got, pt), pt
    for pt in ((23.5, 12.0), (20.0, 13.5)):
        assert not _covered(got, pt), pt


def test_b_mask_keeps_b_cu_only():
    rect = ('  (gr_rect (start 15 8) (end 25 16) (stroke (width 0) (type solid)) '
            '(fill solid) (layer "B.Mask") (uuid "bm"))')
    got = _keepouts(rect)
    assert len(got) == 1 and got[0]['layers'] == {'B.Cu'}, got


if __name__ == '__main__':
    test_unrelated_long_pad_keeps_the_keepout()
    test_broad_opening_touching_a_pad_keeps_the_rest()
    test_pad_own_opening_is_exempt()
    test_circle_is_circumscribed_and_its_hole_inscribed()
    test_arc_stroke_covers_its_true_band()
    test_b_mask_keeps_b_cu_only()
    test_mask_keepouts_are_opt_in()
    test_text_is_a_bounding_box()
    test_board_mask_rect_keeps_copper_out()
    test_logo_footprint_mask_poly_keeps_copper_out()
    test_pad_own_mask_opening_stays_routable()
    print('PASS')
