#!/usr/bin/env python3
"""#1123: the placement graders measure a CUSTOM pad on its copper, not its box.

#1111 made `check_pads` measure a custom (`gr_poly`) pad on the union of its
parsed primitives. Two placement graders still took the pad's size box, which
is symmetric about the anchor and so covers whatever side the copper does
not reach:

  * `legality.occupancy_shape` -- the courtyard united with its pads, so a
    custom pad's empty box side enlarged a part's occupancy (#1094);
  * `legality.pad_copper_overrun_mm` -- pad copper past the outline, so a
    custom pad's empty box corner could read as off the board (#1096).

And `pads_at_pose` carried the box UNCHANGED through a trial rotation while
turning the copper, so at any turn other than a half one the box missed the
copper (tigard JP1/JP2: 0.402 mm2 outside it at +90).

Fixtures are test_1111's L pad -- anchor 0.4 x 0.4 at the origin, a bar x[0, 1]
y[-0.2, 0.2] and an arm x[0.8, 1] y[0.2, 1]; copper x/y in [-0.2, 1], box
+-1.0 -- and a long bar pad (x[0, 2], box +-2 x +-0.2), because a centred
shape makes the box and the copper agree.

Run: python3 -X utf8 tests/test_1123_custom_pad_copper_legality.py [case ...]
"""
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_utils  # noqa: E402

ROOT = run_utils.ROOT_DIR
for _sub in ('py_router', 'py_tools', 'py_placer'):
    _p = os.path.join(ROOT, _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)

RUN_ALL_TIMEOUT = 1200

_L_POLY = ('(gr_poly (pts (xy 0 -0.2) (xy 1 -0.2) (xy 1 1) (xy 0.8 1) '
           '(xy 0.8 0.2) (xy 0 0.2)) (width 0) (fill yes))')
_BAR_POLY = ('(gr_poly (pts (xy 0 -0.2) (xy 2 -0.2) (xy 2 0.2) (xy 0 0.2)) '
             '(width 0) (fill yes))')
#: A zero-area spike: `make_valid` returns the polygon AND a line.
_SPIKE_POLY = ('(gr_poly (pts (xy 0 -0.2) (xy 1 -0.2) (xy 1 0.2) (xy 0.5 0.2) '
               '(xy 0.5 0.8) (xy 0.5 0.2) (xy 0 0.2)) (width 0) (fill yes))')
#: Two primitives that do not touch: the copper is a MultiPolygon.
_SQUARE_POLY = ('(gr_poly (pts (xy 1.5 -0.2) (xy 2 -0.2) (xy 2 0.2) '
                '(xy 1.5 0.2)) (width 0) (fill yes))')
_CURVE = ('(gr_curve (pts (xy 0 0) (xy 0.3 0) (xy 0.6 0.3) (xy 0.6 0.6)) '
          '(width 0.1))')


def _pad(num, prims, rot=0.0, castellated=False, size=0.4):
    prop = ' (property pad_prop_castellated)' if castellated else ''
    return (f'    (pad "{num}" smd custom (at 0 0 {rot:g}) (size {size} {size})'
            f' (layers "F.Cu"){prop} (net 1 "N1")\n'
            f'      (options (clearance outline) (anchor rect))\n'
            f'      (primitives {" ".join(prims)}))\n')


def _rect_pad(num, lx, w, castellated=False):
    prop = ' (property pad_prop_castellated)' if castellated else ''
    return (f'    (pad "{num}" smd rect (at {lx} 0) (size {w} 0.4)'
            f' (layers "F.Cu"){prop} (net 1 "N1"))\n')


def _fp(ref, x, y, pads, rot=0.0, court=None):
    crt = ''
    if court:
        crt = (f'    (fp_rect (start {court[0]} {court[1]}) (end {court[2]} '
               f'{court[3]}) (stroke (width 0.05) (type default)) '
               f'(layer "F.CrtYd"))\n')
    return (f'  (footprint "t:{ref}" (layer "F.Cu") (at {x} {y} {rot:g})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            + crt + pads + '  )\n')


#: Every synthetic board lives under ONE temp root, removed when the run
#: ends (the code review measured 89 leaked t1123* directories).
_ROOT_TMP = tempfile.TemporaryDirectory(prefix='t1123_')


def _board(*footprints):
    wd = tempfile.mkdtemp(dir=_ROOT_TMP.name)
    path = os.path.join(wd, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20240108) (generator pcbnew)\n'
                 '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) '
                 '(44 "Edge.Cuts" user))\n'
                 '  (net 0 "") (net 1 "N1")\n'
                 '  (gr_rect (start 0 0) (end 30 30) (stroke (width 0.1) '
                 '(type default)) (layer "Edge.Cuts"))\n'
                 + ''.join(footprints) + ')\n')
    return path


def _parse(path):
    from kicad_parser import parse_kicad_pcb
    return parse_kicad_pcb(path)


def _overrun(pcb, ref):
    from placement.legality import BoardOutlineGate, pad_copper_overrun_mm
    return pad_copper_overrun_mm(pcb.footprints[ref].pads,
                                 BoardOutlineGate(pcb.board_info, 0.0))


def _occupancy_area(path, ref):
    from placement.legality import graded_parts_from_file
    pcb = _parse(path)
    (g,) = [g for g in graded_parts_from_file(pcb, path) if g.ref == ref]
    assert g.poly is not None, 'no occupancy shape: the courtyard rung failed'
    return g.poly.area


def test_an_l_pad_occupies_its_copper_not_its_box():
    """Courtyard [-0.3, 1.1]^2 holds the L's copper whole, so the occupancy
    IS the courtyard (1.96 mm2); the box (+-1.0) would add the empty side."""
    path = _board(_fp('U1', 10, 10, _pad('1', [_L_POLY]),
                      court=(-0.3, -0.3, 1.1, 1.1)))
    area = _occupancy_area(path, 'U1')
    assert abs(area - 1.96) < 1e-6, area
    print("  L pad occupancy %.4f mm2 (the courtyard; the box read 4.27)"
          % area)


def test_a_box_corner_off_the_board_is_not_copper_off_it():
    """The L at x = 0.5: its copper starts at x 0.3 (on the board), its box
    at -0.5. Gating reads the copper; the RANKING list still reads per-pad
    rects, as documented."""
    from placement.legality import grade_pad_legality
    path = _board(_fp('U1', 0.5, 10, _pad('1', [_L_POLY]),
                      court=(-0.3, -0.3, 1.1, 1.1)))
    pcb = _parse(path)
    assert _overrun(pcb, 'U1') == 0.0, _overrun(pcb, 'U1')
    g = grade_pad_legality(pcb, 0.2, pcb_file=path)
    assert 'U1' not in (g.get('oob_pad_copper_gating_refs') or []), g
    print("  box corner off the board: overrun 0, not gating")


def test_a_pad_the_parser_cannot_draw_falls_back_to_its_box():
    """A `gr_curve` leaves `polygons` None: the box is all there is, and it
    is read, never "clean"."""
    path = _board(_fp('U1', 0.5, 10, _pad('1', [_L_POLY, _CURVE])))
    pcb = _parse(path)
    assert pcb.footprints['U1'].pads[0].polygons is None
    assert abs(_overrun(pcb, 'U1') - 0.5) < 1e-6, _overrun(pcb, 'U1')
    print("  gr_curve pad: box read, overrun 0.5")


def test_a_spiked_primitive_is_measured_on_its_area():
    import check_pads
    path = _board(_fp('U1', 10, 10, _pad('1', [_SPIKE_POLY]),
                      court=(-0.3, -0.3, 1.1, 1.1)))
    pcb = _parse(path)
    cu = check_pads.custom_pad_copper(pcb.footprints['U1'].pads[0])
    assert cu is not None and cu.geom_type == 'Polygon', cu
    assert _overrun(pcb, 'U1') == 0.0
    print("  spike dropped: one polygon, %.4f mm2" % cu.area)


def test_a_quarter_turn_reposes_the_box_and_the_copper():
    """The bar pad's box is 4.0 x 0.4. A trial pose at +90 must read 0.4 x
    4.0 -- what writing it and re-parsing gives -- and the copper, not the
    stale box, decides how far it reaches past the top edge."""
    from shapely.geometry import Polygon
    from shapely.ops import unary_union
    from placement.legality import (BoardOutlineGate, graded_part_at_pose,
                                    pad_copper_overrun_mm, pads_at_pose)
    from placement.writer import write_placed_output
    path = _board(_fp('U1', 10, 1.5, _pad('1', [_BAR_POLY]),
                      court=(-0.3, -0.3, 2.1, 0.3)))
    pcb = _parse(path)
    fp = pcb.footprints['U1']
    gate = BoardOutlineGate(pcb.board_info, 0.0)
    for d in (90.0, 180.0, 270.0):
        posed = pads_at_pose(fp, (fp.x, fp.y, d))
        out = path.replace('.kicad_pcb', '_%d.kicad_pcb' % d)
        write_placed_output(path, out, [{'reference': 'U1', 'new_x': fp.x,
                                         'new_y': fp.y, 'new_rotation': d}])
        rp = _parse(out).footprints['U1'].pads[0]
        assert (round(posed[0].size_x, 6), round(posed[0].size_y, 6)) == (
            round(rp.size_x, 6), round(rp.size_y, 6)), (d, posed[0].size_x,
                                                        posed[0].size_y,
                                                        rp.size_x, rp.size_y)
    # +90 turns the bar toward -y (KiCad's y grows down), 0.5 mm past y = 0.
    over = pad_copper_overrun_mm(pads_at_pose(fp, (fp.x, fp.y, 90.0)), gate)
    assert abs(over - 0.5) < 1e-6, over
    # The occupancy at a trial pose contains the copper posed there.
    for d in (90.0, 270.0):
        poly = graded_part_at_pose(pcb, 'U1', (fp.x, fp.y, d), 'F',
                                   (0, 0, 0, 0), None, False,
                                   pcb_file=path).poly
        cu = unary_union([Polygon(q) for p in pads_at_pose(fp, (fp.x, fp.y, d))
                          for q in p.polygons])
        assert poly is not None and poly.buffer(1e-6).contains(cu), d
    print("  posed boxes equal re-parsed at 90/180/270; the trial-pose "
          "occupancy holds the copper; +90 copper 0.5 mm past the edge")


def test_an_oblique_turn_encloses_the_copper():
    from placement.legality import pads_at_pose
    from placement.writer import write_placed_output
    path = _board(_fp('U1', 10, 10, _pad('1', [_BAR_POLY])))
    fp = _parse(path).footprints['U1']
    for d in (33.0, 45.0, 89.5):
        (p,) = pads_at_pose(fp, (fp.x, fp.y, d))
        # The derived box is axis-aligned: no residual tilt is turned in.
        assert p.rect_rotation == getattr(fp.pads[0], 'rect_rotation', 0.0) \
            == 0.0, (d, p.rect_rotation)
        pts = [pt for poly in p.polygons for pt in poly]
        assert all(abs(u - p.global_x) <= p.size_x / 2 + 1e-9
                   and abs(v - p.global_y) <= p.size_y / 2 + 1e-9
                   for u, v in pts), (d, p.size_x, p.size_y)
        out = path.replace('.kicad_pcb', '_%d.kicad_pcb' % d)
        write_placed_output(path, out, [{'reference': 'U1', 'new_x': fp.x,
                                         'new_y': fp.y, 'new_rotation': d}])
        rp = _parse(out).footprints['U1'].pads[0]
        # gr_poly primitives and a rect anchor: the extent is exact.
        assert abs(p.size_x - rp.size_x) < 1e-4, (d, p.size_x, rp.size_x)
        assert abs(p.size_y - rp.size_y) < 1e-4, (d, p.size_y, rp.size_y)
    print("  33, 45 and 89.5: the posed box encloses the copper, is "
          "untilted, and equals a re-parse")


def test_disjoint_copper_is_every_part():
    """A bar and a square that do not touch: the copper is a MultiPolygon,
    and every part of it is read -- by the helper, the overrun's vertices
    and the occupancy."""
    import check_pads
    from placement.legality import _copper_vertices
    path = _board(_fp('U1', 10, 10, _pad('1', [_BAR_POLY.replace(
        '(xy 2 -0.2) (xy 2 0.2)', '(xy 1 -0.2) (xy 1 0.2)'), _SQUARE_POLY]),
        court=(-0.3, -0.3, 1.1, 0.3)))
    pcb = _parse(path)
    cu = check_pads.custom_pad_copper(pcb.footprints['U1'].pads[0])
    assert cu is not None and cu.geom_type == 'MultiPolygon', cu
    # anchor + bar x[-0.2, 1] (0.48) and the square x[1.5, 2] (0.2)
    assert abs(cu.area - 0.68) < 1e-6, cu.area
    xs = [x for x, _y in _copper_vertices(cu)]
    assert (round(min(xs), 6), round(max(xs), 6)) == (9.8, 12.0), xs
    # courtyard 1.4 x 0.6 holds the bar; the square lies outside it
    area = _occupancy_area(path, 'U1')
    assert abs(area - 1.04) < 1e-6, area
    print("  disjoint copper: %.2f mm2 in two parts, vertices x %.1f..%.1f, "
          "occupancy %.2f" % (cu.area, min(xs), max(xs), area))


def test_castellation_reads_the_copper():
    """(i) a castellated rect pad straddling the edge is exempt; (ii) a
    custom one whose COPPER straddles is exempt; (iii) a custom one whose box
    straddles while its copper is wholly off the board counts -- #1096's own
    rule, applied to the copper."""
    rect = _parse(_board(_fp('U1', 29.9, 10, _rect_pad('1', 0, 0.6, True))))
    assert _overrun(rect, 'U1') == 0.0
    straddle = _parse(_board(_fp('U1', 29.9, 10,
                                 _pad('1', [_L_POLY], castellated=True))))
    assert _overrun(straddle, 'U1') == 0.0
    off = _parse(_board(_fp('U1', 30.25, 10,
                            _pad('1', [_L_POLY], castellated=True))))
    assert abs(_overrun(off, 'U1') - 1.25) < 1e-6, _overrun(off, 'U1')
    print("  castellated: straddling exempt, wholly-off copper 1.25 mm")


def test_the_helper_is_check_pads_copper():
    import check_pads
    pcb = _parse(_board(_fp('U1', 10, 10, _pad('1', [_L_POLY]))))
    p = pcb.footprints['U1'].pads[0]
    assert abs(check_pads.custom_pad_copper(p).area
               - check_pads._copper_geometry(p).area) < 1e-12
    assert check_pads.custom_pad_copper(
        _parse(_board(_fp('U2', 10, 10, _rect_pad('1', 0, 0.6))))
        .footprints['U2'].pads[0]) is None
    print("  custom_pad_copper == check_pads' own copper; None for a rect")


#: Measured at 16c096b1 (run_utils.corpus_boards()): the only tracked boards
#: with a custom pad. A change detector, so a new one is looked at.
CUSTOM_PAD_BOARDS = {'tigard', 'orangecrab_ext_pll',
                     'rp2350_fpga_eensy_prePlane'}


def test_no_tracked_grade_moves():
    """At FILE poses on the tracked corpus, the copper and the box agree on
    every grade: the corpus census measured no change, and this pins it --
    with the helper switched off (the box) and on (the copper)."""
    from unittest.mock import patch
    import check_pads
    from kicad_parser import parse_kicad_pcb
    from placement.legality import grade_body_overlap, grade_pad_legality
    found = set()
    for b in run_utils.corpus_boards():
        pcb = parse_kicad_pcb(b)
        name = os.path.splitext(os.path.basename(b))[0]
        if not any(p.shape == 'custom' for fp in pcb.footprints.values()
                   for p in fp.pads):
            continue
        found.add(name)

        def grades():
            p = parse_kicad_pcb(b)
            g = grade_body_overlap(p, 0.2, pcb_file=b)
            lg = grade_pad_legality(p, 0.2, pcb_file=b)
            return (sorted(tuple(x) for x in g['pairs']),
                    lg['oob_pad_copper_overrun_mm'],
                    lg['oob_pad_copper_gating_refs'])
        on = grades()
        with patch.object(check_pads, 'custom_pad_copper', lambda p: None):
            off = grades()
        assert on == off, name
    assert found == CUSTOM_PAD_BOARDS, found
    print("  %s: every grade identical, copper or box" % sorted(found))


DEMOS = os.environ.get('KICAD_DEMOS_DIR') or next(
    (d for d in (r'C:\Program Files\KiCad\10.0\share\kicad\demos',
                 '/Applications/KiCad/KiCad.app/Contents/SharedSupport/demos',
                 '/usr/share/kicad/demos') if os.path.isdir(d)), None)


def test_kicad_demo_measurements():
    """Where the drawn shape and the box DO disagree at file pose: KiCad
    10's demos (not tracked; skipped without the install). Pinned from
    #1123's census, which predicted them before the fix. jetson's H5-H8
    custom pads are paste-only apertures (F.Paste, no copper layer), which
    both graders skip since #1128 (`_pad_carries_copper`); their overrun was
    0 before that too, so no grade moves on them."""
    if not DEMOS:
        print("  SKIP: no KiCad demos directory (KICAD_DEMOS_DIR)")
        return
    import shutil
    pic = os.path.join(DEMOS, 'pic_programmer', 'pic_programmer.kicad_pcb')
    jet = os.path.join(DEMOS, 'jetson-agx-thor-baseboard',
                       'jetson-agx-thor-baseboard.kicad_pcb')
    for b in (pic, jet):
        if not os.path.isfile(b):
            print("  SKIP: %s absent" % b)
            return
    # Under the run's temp root: the jetson board alone is 88.7 MB.
    wd = tempfile.mkdtemp(dir=_ROOT_TMP.name)
    staged = []
    for b in (pic, jet):
        for ext in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
            src = os.path.splitext(b)[0] + ext
            if os.path.isfile(src):
                shutil.copy(src, wd)
        staged.append(os.path.join(wd, os.path.basename(b)))
    area = _occupancy_area(staged[0], 'JP1')
    assert abs(area - 8.250) < 0.0005, area
    jp = _parse(staged[1])
    for ref in ('H5', 'H6', 'H7', 'H8'):
        assert _overrun(jp, ref) == 0.0, (ref, _overrun(jp, ref))
    print("  pic_programmer JP1 occupancy %.3f (box 8.363); jetson H5-H8 "
          "(paste-only) direct overrun 0 (box 0.888 on H5)" % area)


TESTS = [
    test_an_l_pad_occupies_its_copper_not_its_box,
    test_a_box_corner_off_the_board_is_not_copper_off_it,
    test_a_pad_the_parser_cannot_draw_falls_back_to_its_box,
    test_a_spiked_primitive_is_measured_on_its_area,
    test_a_quarter_turn_reposes_the_box_and_the_copper,
    test_an_oblique_turn_encloses_the_copper,
    test_disjoint_copper_is_every_part,
    test_castellation_reads_the_copper,
    test_the_helper_is_check_pads_copper,
    test_no_tracked_grade_moves,
    test_kicad_demo_measurements,
]


if __name__ == '__main__':
    only = sys.argv[1:]
    ran = 0
    for t in TESTS:
        if only and not any(o in t.__name__ for o in only):
            continue
        print(f"--- {t.__name__}")
        t()
        ran += 1
    if only and not ran:
        print(f"NO TEST matches {only}")
        sys.exit(2)
    print('ALL PASS')
