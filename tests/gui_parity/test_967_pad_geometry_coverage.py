"""Native geometry and text/live-parser coverage on copies of esp_prog (#967).

Plain python3 relaunches into KiCad's python; with no KiCad python at all it
exits 77, never an unmeasured pass.
"""
import copy
import hashlib
import math
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
try:
    import pcbnew
except ImportError:
    sys.path.insert(0, str(ROOT / 'py_router'))
    from headless_plan import reexec_into_kicad
    reexec_into_kicad(__file__)
    print('SKIP: native pad coverage requires KiCad pcbnew')
    raise SystemExit(77)

sys.path[:0] = [str(ROOT/'py_router'), str(ROOT/'py_placer')]
from copy_board import copy_board
from kicad_parser import parse_kicad_pcb, build_pcb_data_from_board
from placement.legality import grade_pad_edge_clearance


def main():
    source = ROOT/'kicad_files/esp_prog.kicad_pcb'
    original = hashlib.sha256(source.read_bytes()).hexdigest()
    with tempfile.TemporaryDirectory(prefix='krt967_native_') as scratch:
        path = Path(scratch)/'board.kicad_pcb'
        copy_board(str(source), str(path))
        board = pcbnew.LoadBoard(str(path))
        fp = next(f for f in board.GetFootprints() if f.GetReference() == 'Y1')
        pad = list(fp.Pads())[0]
        pad.SetShape(pcbnew.PAD_SHAPE_CHAMFERED_RECT)
        pad.SetSize(pcbnew.VECTOR2I(2000000, 1000000))
        pad.SetRoundRectRadiusRatio(.25)
        pad.SetChamferRectRatio(.05)
        pad.SetChamferPositions(15)
        pad.SetOrientation(pcbnew.EDA_ANGLE(33, pcbnew.DEGREES_T))
        pad.SetPosition(pcbnew.VECTOR2I(130000000, 104040000))
        pcbnew.SaveBoard(str(path), board)
        board = pcbnew.LoadBoard(str(path))
        fp = next(f for f in board.GetFootprints() if f.GetReference() == 'Y1')
        pad = list(fp.Pads())[0]
        # Native effective copper, not the footprint bbox or production formula.
        native_bottom = pcbnew.ToMM(pad.GetEffectivePolygon(pcbnew.F_Cu).BBox().GetBottom())
        edge_y = max(pcbnew.ToMM(point.y) for shape in board.GetDrawings()
                     if shape.GetLayer() == pcbnew.Edge_Cuts
                     for point in (shape.GetStart(), shape.GetEnd()))
        gap = edge_y - native_bottom
        assert abs(gap - .523258) < 1e-6, gap
        for parsed in (parse_kicad_pcb(str(path)), build_pcb_data_from_board(board)):
            target = copy.deepcopy(parsed.footprints['Y1'])
            target.pads = [target.pads[0]]
            parsed.footprints = {'Y1': target}
            assert target.pads[0].geometry_approximations == ('chamfered pad',)
            result = grade_pad_edge_clearance(parsed, .55, str(path))
            assert not result['complete'], result
            assert 'chamfered pad' in result['unmeasured'][0]['reason'], result
        # Nonuniform padstacks also flatten per-layer geometry in PCBData.
        pad.SetChamferPositions(0)
        pad.Padstack().SetMode(pcbnew.PADSTACK.MODE_FRONT_INNER_BACK)
        pad.SetShape(pcbnew.F_Cu, pcbnew.PAD_SHAPE_OVAL)
        pad.SetShape(pcbnew.B_Cu, pcbnew.PAD_SHAPE_RECT)
        for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
            pad.SetSize(layer, pcbnew.VECTOR2I(2000000, 1000000))
        pad.SetAttribute(pcbnew.PAD_ATTRIB_PTH)
        pad.SetLayerSet(pcbnew.LSET.AllCuMask())
        pad.SetDrillSize(pcbnew.VECTOR2I(300000, 300000))
        pcbnew.SaveBoard(str(path), board)
        board = pcbnew.LoadBoard(str(path))
        fp = next(f for f in board.GetFootprints() if f.GetReference() == 'Y1')
        pad = list(fp.Pads())[0]
        back_gap = edge_y - pcbnew.ToMM(
            pad.GetEffectivePolygon(pcbnew.B_Cu).BBox().GetBottom())
        assert abs(back_gap - .496026) < 1e-6, back_gap
        for parsed in (parse_kicad_pcb(str(path)), build_pcb_data_from_board(board)):
            target = parsed.footprints['Y1'].pads[0]
            assert 'per-layer padstack' in target.geometry_approximations, target
            result = grade_pad_edge_clearance(parsed, .55, str(path))
            assert not result['complete'], result
            assert any('per-layer padstack' in row['reason']
                       for row in result['unmeasured']), result
        print(f'PASS: native chamfer gap {gap:.6f}, padstack gap {back_gap:.6f} mm < .55 mm; '
              'both parsers disclose chamfer/padstack coverage')
        # The routing broad phase snaps within one degree of cardinal axes.
        # Exact edge grading must recover the actual copper tilt in both parsers.
        for angle in (0, .5, 1, 1.0001, 89.5, 90, 90.5, 179.5, 269.5, 359.5):
            board = pcbnew.LoadBoard(str(source))
            fp = next(f for f in board.GetFootprints() if f.GetReference() == 'Y1')
            pad = list(fp.Pads())[0]
            pad.SetShape(pcbnew.PAD_SHAPE_RECT)
            pad.SetSize(pcbnew.VECTOR2I(2000000, 1000000))
            pad.SetOrientation(pcbnew.EDA_ANGLE(angle, pcbnew.DEGREES_T))
            pad.SetPosition(pcbnew.VECTOR2I(130000000, 104000000))
            pcbnew.SaveBoard(str(path), board)
            board = pcbnew.LoadBoard(str(path))
            expected = 1.5 - (abs(math.sin(math.radians(angle)))
                              + .5 * abs(math.cos(math.radians(angle))))
            for parsed in (parse_kicad_pcb(str(path)), build_pcb_data_from_board(board)):
                target = parsed.footprints['Y1']
                target.pads = [target.pads[0]]
                parsed.footprints = {'Y1': target}
                result = grade_pad_edge_clearance(parsed, .55, str(path))
                assert result['complete'], result
                assert abs(result['minimum_gap_mm'] - expected) < 1e-6, (angle, result)
        pad = list(next(f for f in board.GetFootprints() if f.GetReference() == 'Y1').Pads())[0]
        pad.SetShape(pcbnew.PAD_SHAPE_CIRCLE)
        pad.SetSize(pcbnew.VECTOR2I(2000000, 1000000))
        pcbnew.SaveBoard(str(path), board)
        board = pcbnew.LoadBoard(str(path))
        for parsed in (parse_kicad_pcb(str(path)), build_pcb_data_from_board(board)):
            result = grade_pad_edge_clearance(parsed, .55, str(path))
            assert not result['complete'], result
            assert any('unequal-axis circle' in row['reason'] for row in result['unmeasured'])
        print('PASS: near-cardinal exact tilt and unequal-axis circle coverage')
        padstack_arm(Path(scratch))
        custom_size_arm(Path(scratch))
        custom_copper_arm(Path(scratch))
    assert hashlib.sha256(source.read_bytes()).hexdigest() == original


SYN_HEAD = ('(kicad_pcb (version 20241229) (generator "pcbnew") (general (thickness 1.6))\n'
            ' (paper "A4") (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal)'
            ' (31 "B.Cu" signal) (34 "B.Paste" user) (35 "F.Paste" user) (38 "B.Mask" user)'
            ' (39 "F.Mask" user) (44 "Edge.Cuts" user))\n (setup)\n (net 0 "") (net 1 "/A")\n'
            ' (gr_rect (start 0 0) (end 60 40) (stroke (width 0.1) (type solid)) (fill none)'
            ' (layer "Edge.Cuts"))\n')


def _synthetic(scratch, name, footprints):
    path = scratch/f'{name}.kicad_pcb'
    path.write_text(SYN_HEAD + footprints + ')\n', encoding='utf-8')
    board = pcbnew.LoadBoard(str(path))
    assert board is not None, f'pcbnew refused the {name} fixture'
    return path, board


def padstack_arm(scratch):
    """A KiCad 9 padstack's per-layer size counts only on a layer the pad has
    COPPER on. mokya_lora_phone's SW1 pads are 0.65 mm on F.Cu, their only
    copper, with 1.905 x 1.27 inner/back entries: the text parser read them
    as 1.905 x 1.27. A pad on B.Cu only is its B.Cu entry, where the pcbnew
    builder read the F.Cu base. Each pad is checked against native
    `PAD.GetSize(layer)` on every copper layer it is on."""
    stack = ('(padstack (mode front_inner_back) (layer "Inner" (shape roundrect)'
             ' (size 1.905 1.27) (roundrect_rratio 0.15)) (layer "B.Cu" (shape'
             ' roundrect) (size %s) (roundrect_rratio 0.15)))')
    pads = ''.join(
        f'  (pad "{n}" {kind} roundrect (at {x} 0) (size 0.65 0.65){drill}'
        f' (layers {lys}) (roundrect_rratio 0.15) (net 1 "/A") {stack % back})\n'
        for n, kind, x, drill, lys, back in (
            ('1', 'smd', 0, '', '"F.Cu" "F.Mask" "F.Paste"', '1.905 1.27'),
            ('2', 'smd', 4, '', '"B.Cu" "B.Mask" "B.Paste"', '1.2 0.9'),
            ('3', 'thru_hole', 8, ' (drill 0.4)', '"*.Cu" "*.Mask"', '1.2 0.9')))
    path, board = _synthetic(scratch, 'padstack', (
        ' (footprint "L:PS" (layer "F.Cu") (at 20 20)\n'
        '  (property "Reference" "U1" (at 0 -3 0) (layer "F.Cu") (hide yes)'
        ' (effects (font (size 1 1) (thickness 0.15))))\n' + pads + ' )\n'))
    cu_ids = {pcbnew.F_Cu, pcbnew.B_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu}
    native = {}
    for pad in next(iter(board.GetFootprints())).Pads():
        on = [l for l in pad.GetLayerSet().CuStack() if l in cu_ids]
        native[pad.GetNumber()] = (max(pcbnew.ToMM(pad.GetSize(l).x) for l in on),
                                   max(pcbnew.ToMM(pad.GetSize(l).y) for l in on))
    assert native == {'1': (.65, .65), '2': (1.2, .9), '3': (1.905, 1.27)}, native
    for label, parsed in (('text', parse_kicad_pcb(str(path))),
                          ('pcbnew', build_pcb_data_from_board(board))):
        got = {p.pad_number: (round(p.size_x, 6), round(p.size_y, 6))
               for p in parsed.footprints['U1'].pads}
        assert got == native, (label, got, native)
    print('PASS: padstack sizes are the max over the pad\'s own copper layers, '
          f'on both parsers: {native}')


def custom_size_arm(scratch):
    """A custom pad's `size_x/size_y/rect_rotation` on both parsers, against
    pcbnew's own copper.

    - Copper that IS the anchor (no primitive, or every primitive inside it)
      is the anchor rect with its residual tilt: A (len42_filter2 SW1.3, a
      4.572 x 0.406 anchor at 333 deg with a copper offset) and D (a
      primitive inside a 2 x 1 anchor at 30 deg). The pcbnew builder sized A
      from GetBoundingBox(), centred on the HOLE: 5.16 x 2.46.
    - Real primitives give the board-frame box about the COPPER centre, rr 0:
      B (awkb_36_split SW16.1, a gr_poly on a circle anchor at 345 deg), where
      the text parser counted the circle anchor's bounding-rect corners.
    - An NPTH custom pad is its anchor, not its bigger drill: C (usmu_smu J2,
      a 0.6 anchor with a 0.65 drill), which the pcbnew builder sized 0.65."""
    pads = (
        '  (pad "A" smd custom (at 0 0 333) (size 4.572 0.4064) (drill (offset -0.4064 0.19525))'
        ' (layers "F.Cu") (net 1 "/A") (options (clearance outline) (anchor rect))'
        ' (primitives))\n'
        '  (pad "B" smd custom (at 12 0 345) (size 2.4892 2.4892) (layers "F.Cu" "F.Mask")'
        ' (net 1 "/A") (options (clearance outline) (anchor circle)) (primitives (gr_poly'
        ' (pts (xy -1.2573 -1.194816) (xy -1.207516 -1.2446) (xy 1.207516 -1.2446)'
        ' (xy 1.2573 -1.194816) (xy 1.2573 0.74676) (xy 0.75946 1.2446) (xy -1.207516 1.2446)'
        ' (xy -1.2573 1.194816)) (width 0) (fill yes))))\n'
        '  (pad "" np_thru_hole custom (at 20 0 270) (size 0.6 0.6) (drill 0.65)'
        ' (layers "*.Cu" "*.Mask") (options (clearance outline) (anchor circle)) (primitives))\n'
        '  (pad "D" smd custom (at 26 0 30) (size 2 1) (layers "F.Cu") (net 1 "/A")'
        ' (options (clearance outline) (anchor rect)) (primitives (gr_poly (pts'
        ' (xy -0.5 -0.3) (xy 0.5 -0.3) (xy 0.5 0.3) (xy -0.5 0.3)) (width 0.1) (fill yes))))\n')
    path, board = _synthetic(scratch, 'custom_size', (
        ' (footprint "L:CS" (layer "F.Cu") (at 15 20)\n'
        '  (property "Reference" "U2" (at 0 -3 0) (layer "F.Cu") (hide yes)'
        ' (effects (font (size 1 1) (thickness 0.15))))\n' + pads + ' )\n'))
    live = list(next(iter(board.GetFootprints())).Pads())
    text = parse_kicad_pcb(str(path)).footprints['U2'].pads
    gui = build_pcb_data_from_board(board).footprints['U2'].pads
    assert len(live) == len(text) == len(gui) == 4, (len(live), len(text), len(gui))
    for lp, tp, gp in zip(live, text, gui):
        tag = lp.GetNumber() or 'npth'
        # Native copper: the effective polygon about the shape (copper) centre.
        ol = lp.GetEffectivePolygon(pcbnew.F_Cu)
        c = lp.ShapePos(pcbnew.F_Cu)
        pts = [(pcbnew.ToMM(ol.COutline(o).CPoint(i).x - c.x),
                pcbnew.ToMM(ol.COutline(o).CPoint(i).y - c.y))
               for o in range(ol.OutlineCount()) for i in range(ol.COutline(o).PointCount())]
        ext = (2 * max(abs(x) for x, _ in pts), 2 * max(abs(y) for _, y in pts))
        for label, p in (('text', tp), ('pcbnew', gp)):
            th = math.radians(p.rect_rotation)
            box = (p.size_x * abs(math.cos(th)) + p.size_y * abs(math.sin(th)),
                   p.size_x * abs(math.sin(th)) + p.size_y * abs(math.cos(th)))
            # The model rect's board-axis extent IS pcbnew's copper extent
            # (pcbnew's polygon sits inside arcs by its 5 um max error).
            assert all(abs(a - b) < .006 for a, b in zip(box, ext)), (tag, label, box, ext)
        assert (abs(tp.size_x - gp.size_x) < .006 and abs(tp.size_y - gp.size_y) < .006
                and abs(tp.rect_rotation - gp.rect_rotation) < 1e-6), (
            tag, (tp.size_x, tp.size_y, tp.rect_rotation), (gp.size_x, gp.size_y, gp.rect_rotation))
    by = {(p.pad_number or 'npth'): (round(p.size_x, 4), round(p.size_y, 4), round(p.rect_rotation, 2))
          for p in text}
    assert by['A'] == (4.572, .4064, 27.0) and by['D'] == (2.0, 1.0, -30.0), by
    assert by['B'][2] == 0.0 and by['npth'] == (0.6, 0.6, 0.0), by
    print(f'PASS: custom-pad size/tilt agree on both parsers and with pcbnew\'s copper: {by}')


def custom_copper_arm(scratch):
    """A custom pad's `polygons` reach exactly as far as pcbnew's copper, on
    both parsers -- the obstacle map and check_drc read them before the size
    rect, and before this nothing compared them.

    KiCad strokes a primitive with ROUND caps and joins: the text parser's
    square caps and mitered corners reached past it (qfhmix01 D3: 0.083 mm
    on a 0.4 mm arc stroke, laser_backplane_dvi J8.2: 0.022 mm on a stroked
    poly), its inscribed 32-gon fell 13 um inside a 2.75 mm ring
    (kirdy_laser_drv H1), and it drew a KiCad 9 rounded-rect primitive with
    sharp corners (rp2350_fpga_eensy U5: 21 um). Measured both ways at every vertex and edge
    midpoint: text copper against pcbnew's own polygon (its distance query),
    pcbnew's vertices against check_drc's reading of the text polygons, to
    compare_pcb_data's 0.01 mm. The residue that bound admits is two chord
    errors added: pcbnew's polygon sits up to its 5 um max error inside a
    curve, and the text ring's inscribed inner edge up to its own sagitta
    into the hole (8.6 um on the 2.25 mm inner edge here). Before the fix the
    text copper of the four pads missed by 83, 21, 65 and 13 um."""
    pads = (
        # qfhmix01 D3.1: a wide arc stroke on a 1 um anchor, at 315 deg
        '  (pad "1" smd custom (at 0 0 315) (size 0.001 0.001) (layers "F.Cu") (net 1 "/A")'
        ' (options (clearance outline) (anchor rect)) (primitives (gr_arc (start 0.404306'
        ' 0.548105) (mid 0.307348 0.003177) (end 0.402138 -0.542133) (width 0.4)) (gr_poly'
        ' (pts (xy 0.405 0.75) (xy -0.645 0.75) (xy -0.645 -0.75) (xy 0.405 -0.75))'
        ' (width 0) (fill yes))))\n'
        # laser_backplane_dvi J8.2: a stroked poly with sharp corners plus an arc
        '  (pad "2" smd custom (at 6 0 90) (size 1 1) (layers "F.Cu") (net 1 "/A")'
        ' (options (clearance outline) (anchor circle)) (primitives (gr_poly (pts'
        ' (xy -0.55 0.55) (xy -0.55 -2) (xy -0.3 -2) (xy -0.3 -1.4) (xy -0.1 -1)'
        ' (xy 0.1 -0.552) (xy 0.7 0) (xy 1.6 0.3) (xy 2 0.3) (xy 2 0.55)) (width 0.1)'
        ' (fill yes)) (gr_arc (start 2 0.3) (mid 0.373654 -0.373654) (end -0.3 -2)'
        ' (width 0.1))))\n'
        # a line and a stroked rect at angles no axis-aligned box hides
        '  (pad "3" smd custom (at 12 0 30) (size 0.3 0.3) (layers "F.Cu") (net 1 "/A")'
        ' (options (clearance outline) (anchor circle)) (primitives (gr_line (start 0 0)'
        ' (end 1.5 0.4) (width 0.3)) (gr_rect (start -1 -0.6) (end -0.2 0.6) (width 0.2))))\n'
        # kirdy_laser_drv H1: a 2.75 mm unfilled ring
        '  (pad "4" smd custom (at 20 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "/A")'
        ' (options (clearance outline) (anchor circle)) (primitives (gr_circle'
        ' (center 0 0) (end 2.5 0) (width 0.5) (fill no))))\n'
        # rp2350_fpga_eensy U5: a KiCad 9 rounded-rect primitive, here at 30 deg
        '  (pad "5" smd custom (at 26 0 30) (size 0.3 0.3) (layers "F.Cu") (net 1 "/A")'
        ' (options (clearance outline) (anchor circle)) (primitives (gr_rect'
        ' (start 0.05 -0.5) (end 0.3 0.15) (radius 0.05) (width 0) (fill yes))'
        ' (gr_rect (start -1 -0.5) (end -0.4 0.5) (radius 0.2) (width 0.1) (fill yes))))\n')
    path, board = _synthetic(scratch, 'custom_copper', (
        ' (footprint "L:CC" (layer "F.Cu") (at 15 20)\n'
        '  (property "Reference" "U3" (at 0 -4 0) (layer "F.Cu") (hide yes)'
        ' (effects (font (size 1 1) (thickness 0.15))))\n' + pads + ' )\n'))
    from check_drc import _point_to_polys_distance
    from kicad_parser import compare_pcb_data
    live = list(next(iter(board.GetFootprints())).Pads())
    parsed = {'text': parse_kicad_pcb(str(path)), 'pcbnew': build_pcb_data_from_board(board)}
    worst = {}
    for i, lp in enumerate(live):
        native = lp.GetEffectivePolygon(pcbnew.F_Cu)
        nat_pts = [(pcbnew.ToMM(native.COutline(o).CPoint(k).x),
                    pcbnew.ToMM(native.COutline(o).CPoint(k).y))
                   for o in range(native.OutlineCount())
                   for k in range(native.COutline(o).PointCount())]
        for label, pcb in parsed.items():
            polys = pcb.footprints['U3'].pads[i].polygons
            assert polys, (lp.GetNumber(), label, 'no polygons')
            samples = [p for r in polys for p in r] + [
                ((r[k][0] + r[(k + 1) % len(r)][0]) / 2, (r[k][1] + r[(k + 1) % len(r)][1]) / 2)
                for r in polys for k in range(len(r))]
            out = max(math.sqrt(native.SquaredDistance(pcbnew.VECTOR2I(
                pcbnew.FromMM(x), pcbnew.FromMM(y)))) / 1e6 for x, y in samples)
            short = max(_point_to_polys_distance(x, y, polys) for x, y in nat_pts)
            worst[(lp.GetNumber(), label)] = (round(out, 4), round(short, 4))
            assert out < .01 and short < .01, (lp.GetNumber(), label, out, short)
    diffs = [d for d in compare_pcb_data(parsed['pcbnew'], parsed['text']) if 'polygons' in d]
    assert not diffs, diffs[:3]
    print('PASS: custom-pad copper matches pcbnew\'s within its max error, both ways, '
          f'on both parsers (worst past, worst short): {worst}')


if __name__ == '__main__':
    main()
