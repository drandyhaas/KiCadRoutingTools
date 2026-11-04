#!/usr/bin/env python3
"""main's parse-path parity on the IPC read path: the kipy builder against the
text parser, field by field, on one board both of them read.

main's SWIG builder gained these since the last sync, and the kipy builder
(`build_pcb_data_from_board` on this branch) is where the IPC plugin reads its
board, so each is ported there and graded here:

  A. #1268  via type ('through'/'blind'/'buried'/'micro'), the layer pair top
            first (6552e70f), an untyped via read F.Cu/B.Cu (db60fe77), and
            the write->read round trip through the adapter's writer
  B. #1279  every track's and via's uuid, the live `sense:` groups
            (`kipy_sense_groups`) and the resolved branches with their holds
  C. #1164  zones: teardrop areas flagged, holes, arc outline vertices
            (fe367f3f), the closing repeat merged (37fb1b93), mask zones and
            rule areas NOT modelled as pours (da63ad5c), footprint zones;
            #1329 each zone's pad connection and thermal spokes
  D. #967   pads: the PadStackShape name (every kipy pad used to read 'rect'),
            the pad kind, the layer SET (4f3e4e6e), the size over the copper
            layers the pad is on (62aace71); #1248 die length / delay; the
            zone_connect override, the pad's own else its footprint's (a pad
            with NONE is not served by the pour, 0ef3ebd6)
  E. #1211  footprint sheet name / file; #481 the Reference label; net ties;
            the footprint's zone_connect, and compare_pcb_data diffs it
  F. #304   Edge.Cuts as main's pcbnew scan reads it -- lines (the old kipy
            reader matched none), a gr_poly with an arc and a repeated closing
            point (6febcfad), a footprint-drawn window -- classified into
            outers and cut-outs, and #829's outline ownership
  G. #337 / #908  copper graphics: board shapes, a pad-bearing footprint's
            copper, a pad-less logo's copper left out, a polygon's arc vertex
            (6925b417), an unfilled zero-width shape at pcbnew's 0.1 mm
  J.        a custom pad's copper drawn to one side of its anchor carries
            Pad.copper_box, read off the rendered polygon; one drawn about
            its anchor carries none
  K.        main's round since 83646c8b: #1450 a chamfered-rect pad carries
            its cut-corner outline in Pad.polygons; #1409 a footprint's closed
            Edge.Cuts window that spares its own pads is carried at ANY pose
            (here off the board, where containment alone calls it
            structural); #1404 an Edge.Cuts circle is linearised at
            EDGE_CHORD_EPS, the text parser's vertex for vertex

WHY A HAND-BUILT kipy BOARD. The builder reaches a RUNNING KiCad over a
socket, and `fake_ipc_board` serves reads by re-parsing the file, which would
grade the text parser against itself. So the board here is built from REAL
kipy objects (`kipy.board_types`, edited through the proto fields KiCad fills)
carrying the same numbers as the fixture's text, and the oracle is
`parse_kicad_pcb` on that text. What it does NOT prove: that a live KiCad
serializes each item exactly so (e.g. a loaded zone outline's arc as one arc
node) -- nothing here talks to KiCad.

    python3 -X utf8 tests/test_1279_kipy_builder_parity.py
"""
import math
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

try:
    from kipy.board_types import (BoardCircle, BoardPolygon, BoardRectangle,
                                  BoardSegment, FootprintInstance, Group, Net,
                                  Pad, Track, Via, Zone)
    from kipy.geometry import (ArcStartMidEnd, PolygonWithHoles, PolyLine,
                               PolyLineNode, Vector2)
    from kipy.proto.board import board_types_pb2 as bt
except Exception as _e:                                          # noqa: BLE001
    print(f"SKIP: kipy is not importable ({type(_e).__name__}: {_e}); install "
          f"kicad-python, which requirements.txt lists on this branch")
    sys.exit(77)

import kicad_ipc_adapter as adapter                              # noqa: E402
import kicad_parser as kp                                        # noqa: E402
from sense_branch import branch_signature, sense_groups_from_text  # noqa: E402

FAILS = []


def check(cond, msg):
    print(f"  {'ok  ' if cond else 'FAIL'}  {msg}")
    if not cond:
        FAILS.append(msg)
    return cond


# ---------------------------------------------------------------------------
# The fixture: one board, as text (the oracle) and as kipy objects (the IPC
# input). Every number below appears in both.
# ---------------------------------------------------------------------------
U = {k: f"{k * 8}-{k * 4}-{k * 4}-{k * 4}-{k * 12}" for k in "0123456789abcdef"}
U_T1, U_T2, U_V1, U_V2, U_V3, U_V4, U_V5 = (U['1'], U['2'], U['3'], U['4'],
                                            U['5'], U['6'], U['7'])
U_G, U_Z1, U_Z2, U_FP, U_FPZ, U_GP, U_GL = (U['8'], U['9'], U['a'], U['b'],
                                            U['c'], U['d'], U['e'])

BOARD = f'''(kicad_pcb (version 20260206) (generator "pcbnew")
  (general (thickness 1.6))
  (layers
    (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal) (2 "B.Cu" signal)
    (13 "F.Paste" user) (1 "F.Mask" user) (3 "B.Mask" user) (5 "F.SilkS" user)
    (25 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "") (net 1 "GND") (net 2 "SIG")
  (footprint "lib:U" (layer "F.Cu") (uuid "{U_FP}") (at 30 20 90)
    (property "Reference" "U1" (at 0 -2 90) (layer "F.SilkS")
      (effects (font (size 1 1.2) (thickness 0.15)) (justify left)))
    (sheetname "/CH2/") (sheetfile "ch.kicad_sch")
    (zone_connect 2)
    (net_tie_pad_groups "1, 2")
    (pad "1" smd circle (at 0 0 90) (size 1 1) (layers "F.Cu" "F.Mask" "F.Paste")
      (net 2 "SIG") (die_length 1.5) (die_delay 0.012345) (zone_connect 0))
    (pad "2" thru_hole circle (at 2 0 90) (size 1.6 1.6) (drill 0.8)
      (layers "*.Cu" "*.Mask") (net 1 "GND"))
    (pad "" np_thru_hole circle (at 4 0 90) (size 2 2) (drill 2) (layers "*.Cu" "*.Mask"))
    (pad "3" smd rect (at -3 0 90) (size 1 1) (layers "B.Cu" "B.Mask")
      (padstack (mode front_inner_back)
        (layer "Inner" (shape rect) (size 2 2))
        (layer "B.Cu" (shape rect) (size 1.5 0.8)))
      (net 2 "SIG") (zone_connect 3))
    (fp_line (start -1 3) (end 1 3) (stroke (width 0.2) (type solid)) (layer "F.Cu"))
    (fp_rect (start 5 -1) (end 7 1) (stroke (width 0.1) (type solid)) (fill none)
      (layer "Edge.Cuts"))
    (zone (net 1) (net_name "GND") (layer "B.Cu") (uuid "{U_FPZ}")
      (connect_pads (clearance 0.2)) (min_thickness 0.25)
      (polygon (pts (xy 31 21) (xy 33 21) (xy 33 23)))))
  (footprint "lib:R" (layer "F.Cu") (at 30 25)
    (property "Reference" "R1" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu" "F.Mask" "F.Paste")
      (net 2 "SIG")))
  (footprint "lib:LOGO" (layer "F.Cu") (at 45 5)
    (property "Reference" "G1" (at 0 0) (layer "F.SilkS"))
    (fp_poly (pts (xy 0 0) (xy 1 0) (xy 1 1)) (stroke (width 0) (type solid))
      (fill solid) (layer "F.Cu")))
  (gr_line (start 0 0) (end 50 0) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts"))
  (gr_line (start 50 0) (end 50 40) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts"))
  (gr_line (start 50 40) (end 0 40) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts"))
  (gr_line (start 0 40) (end 0 0) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts"))
  (gr_poly (pts (xy 10 20) (xy 14 20) (arc (start 14 20) (mid 15 22) (end 14 24))
      (xy 10 24) (xy 10 20))
    (stroke (width 0.1) (type solid)) (fill none) (layer "Edge.Cuts"))
  (gr_poly (pts (xy 2 2) (xy 6 2) (arc (start 6 2) (mid 7 4) (end 6 6)) (xy 2 6))
    (stroke (width 0) (type solid)) (fill solid) (layer "F.Cu") (net 1) (uuid "{U_GP}"))
  (gr_line (start 2 10) (end 8 10) (stroke (width 0.25) (type solid)) (layer "B.Cu")
    (uuid "{U_GL}"))
  (gr_rect (start 10 30) (end 12 32) (stroke (width 0) (type solid)) (fill none)
    (layer "F.Cu"))
  (gr_circle (center 15 35) (end 16 35) (stroke (width 0.1) (type solid)) (fill solid)
    (layer "F.Cu"))
  (segment (start 30 20) (end 30 25) (width 0.25) (layer "F.Cu") (net 2) (uuid "{U_T1}"))
  (segment (start 20 10) (end 24 10) (width 0.25) (layer "F.Cu") (net 2) (uuid "{U_T2}"))
  (via (at 20 10) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 2) (uuid "{U_V1}"))
  (via blind (at 21 10) (size 0.6) (drill 0.3) (layers "In1.Cu" "F.Cu") (net 2) (uuid "{U_V2}"))
  (via micro (at 22 10) (size 0.3) (drill 0.1) (layers "F.Cu" "In1.Cu") (net 2) (uuid "{U_V3}"))
  (via buried (at 23 10) (size 0.6) (drill 0.3) (layers "In2.Cu" "In1.Cu") (net 2) (uuid "{U_V4}"))
  (via (at 24 10) (size 0.6) (drill 0.3) (layers "In1.Cu" "In2.Cu") (net 2) (uuid "{U_V5}"))
  (zone (net 1) (net_name "GND") (layers "F.Cu" "B.Cu") (uuid "{U_Z1}") (priority 2)
    (connect_pads (clearance 0.3)) (min_thickness 0.2)
    (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.4) (island_removal_mode 2)
      (island_area_min 10))
    (polygon (pts (xy 1 1) (xy 30 1) (xy 30 30) (arc (start 30 30) (mid 15 35) (end 1 30))
      (xy 1 1)))
    (polygon (pts (xy 5 5) (xy 8 5) (xy 8 8))))
  (zone (net 2) (net_name "SIG") (layer "F.Cu") (uuid "{U_Z2}")
    (attr (teardrop (type padvia))) (connect_pads yes (clearance 0)) (min_thickness 0.1)
    (polygon (pts (xy 40 10) (xy 41 10) (xy 41 11))))
  (zone (net 0) (net_name "") (layer "F.Mask")
    (polygon (pts (xy 40 30) (xy 41 30) (xy 41 31))))
  (zone (net 0) (net_name "") (layers "F.Cu") (name "KO")
    (keepout (tracks not_allowed) (vias not_allowed) (pads allowed)
      (copperpour not_allowed) (footprints allowed))
    (polygon (pts (xy 42 30) (xy 43 30) (xy 43 31))))
  (group "sense:U1.1" (uuid "{U_G}") (members "{U_T1}"))
)
'''


def nm(mm):
    return int(round(mm * 1e6))


def vec(x, y):
    return Vector2.from_xy(nm(x), nm(y))


def net(name):
    n = Net()
    n.proto.name = name
    return n


NETS = {'GND': net('GND'), 'SIG': net('SIG')}


def polyline(nodes):
    """nodes: (x, y) points and ('arc', (sx, sy), (mx, my), (ex, ey))."""
    pl = PolyLine()
    for n in nodes:
        if n[0] == 'arc':
            a = ArcStartMidEnd()
            a.start, a.mid, a.end = vec(*n[1]), vec(*n[2]), vec(*n[3])
            node = PolyLineNode()
            node.arc = a
            pl.append(node)
        else:
            pl.append(PolyLineNode.from_xy(nm(n[0]), nm(n[1])))
    return pl


def pwh(nodes, holes=()):
    p = PolygonWithHoles()
    p.outline = polyline(nodes)
    for h in holes:
        p.add_hole(polyline(h))
    return p


def seg_shape(a, b, layer, width=0.1, uuid=None):
    s = BoardSegment()
    s.start, s.end, s.layer = vec(*a), vec(*b), layer
    s.attributes.stroke.width = nm(width)
    if uuid:
        s.proto.id.value = uuid
    return s


def poly_shape(nodes, layer, width, filled, net_name=None, uuid=None):
    s = BoardPolygon()
    s.polygons.append(pwh(nodes))
    s._pack()
    s.layer = layer
    s.attributes.stroke.width = nm(width)
    s.attributes.fill.filled = filled
    if net_name:
        s.net = NETS[net_name]
    if uuid:
        s.proto.id.value = uuid
    return s


def rect_shape(a, b, layer, width, filled):
    s = BoardRectangle()
    s.top_left, s.bottom_right, s.layer = vec(*a), vec(*b), layer
    s.attributes.stroke.width = nm(width)
    s.attributes.fill.filled = filled
    return s


def circle_shape(c, e, layer, width, filled):
    s = BoardCircle()
    s.center, s.radius_point, s.layer = vec(*c), vec(*e), layer
    s.attributes.stroke.width = nm(width)
    s.attributes.fill.filled = filled
    return s


def g(fx, fy, frot, x, y):
    """A footprint-local point at the footprint's pose (the file's frame)."""
    return kp.local_to_global(fx, fy, frot, x, y)


def make_pad(number, pos, ptype, net_name, layers, shape, size, drill=None,
             extra_layers=()):
    p = Pad()
    p.number = number
    p.position = vec(*pos)
    p.pad_type = ptype
    if net_name:
        p.net = NETS[net_name]
    p.padstack.layers = layers
    cl = p.padstack.proto.copper_layers[0]
    cl.layer = bt.BL_F_Cu
    cl.shape = shape
    cl.size.x_nm, cl.size.y_nm = nm(size[0]), nm(size[1])
    for lyr, sz in extra_layers:
        e = p.padstack.proto.copper_layers.add()
        e.layer, e.shape = lyr, bt.PSS_RECTANGLE
        e.size.x_nm, e.size.y_nm = nm(sz[0]), nm(sz[1])
    if drill:
        p.padstack.drill.diameter = vec(drill, drill)
    return p


class FakeBoard:
    """The kipy Board calls the builder makes, over the objects built below."""

    def __init__(self):
        self.nets = [NETS['GND'], NETS['SIG']]
        self.tracks, self.vias, self.zones = [], [], []
        self.shapes, self.fps, self.groups = [], [], []

    def get_nets(self):
        return self.nets

    def get_tracks(self):
        return self.tracks

    def get_vias(self):
        return self.vias

    def get_zones(self):
        return self.zones

    def get_shapes(self):
        return self.shapes

    def get_footprints(self):
        return self.fps

    def get_groups(self):
        return self.groups

    def get_enabled_layers(self):
        return [bt.BL_F_Cu, bt.BL_In1_Cu, bt.BL_In2_Cu, bt.BL_B_Cu,
                bt.BL_F_Paste, bt.BL_F_Mask, bt.BL_B_Mask, bt.BL_F_SilkS,
                bt.BL_Edge_Cuts]

    def get_stackup(self):
        raise RuntimeError("no stackup in this fixture")


def build_kipy_board():
    b = FakeBoard()
    for uuid, a, c in ((U_T1, (30, 20), (30, 25)), (U_T2, (20, 10), (24, 10))):
        t = Track()
        t.start, t.end, t.width, t.layer = vec(*a), vec(*c), nm(0.25), bt.BL_F_Cu
        t.net = NETS['SIG']
        t.proto.id.value = uuid
        b.tracks.append(t)
    for uuid, x, vt, span, size, drill in (
            (U_V1, 20, bt.VT_THROUGH, (bt.BL_F_Cu, bt.BL_B_Cu), 0.6, 0.3),
            (U_V2, 21, bt.VT_BLIND, (bt.BL_In1_Cu, bt.BL_F_Cu), 0.6, 0.3),
            (U_V3, 22, bt.VT_MICRO, (bt.BL_F_Cu, bt.BL_In1_Cu), 0.3, 0.1),
            (U_V4, 23, bt.VT_BURIED, (bt.BL_In2_Cu, bt.BL_In1_Cu), 0.6, 0.3),
            (U_V5, 24, bt.VT_THROUGH, (bt.BL_In1_Cu, bt.BL_In2_Cu), 0.6, 0.3)):
        v = Via()
        v.position = vec(x, 10)
        v.diameter, v.drill_diameter = nm(size), nm(drill)
        v.net = NETS['SIG']
        v.type = vt
        v.padstack.drill.start_layer, v.padstack.drill.end_layer = span
        v.proto.id.value = uuid
        b.vias.append(v)

    z = Zone()
    z.layers = [bt.BL_F_Cu, bt.BL_B_Cu]
    z.net = NETS['GND']
    z.outline = pwh([(1, 1), (30, 1), (30, 30), ('arc', (30, 30), (15, 35), (1, 30)),
                     (1, 1)], holes=[[(5, 5), (8, 5), (8, 8)]])
    z.priority = 2
    z.island_mode = bt.IRM_AREA
    z.min_island_area = 10 * 10**12
    z.clearance, z.min_thickness = nm(0.3), nm(0.2)
    # #1329: the text's (connect_pads (clearance X)) is a thermal joint, its
    # (fill (thermal_gap) (thermal_bridge_width)) the spokes.
    z.proto.copper_settings.connection.zone_connection = bt.ZCS_THERMAL
    z.proto.copper_settings.connection.thermal_spokes.gap.value_nm = nm(0.5)
    z.proto.copper_settings.connection.thermal_spokes.width.value_nm = nm(0.4)
    z.proto.id.value = U_Z1
    b.zones.append(z)
    td = Zone()
    td.layers = [bt.BL_F_Cu]
    td.net = NETS['SIG']
    td.outline = pwh([(40, 10), (41, 10), (41, 11)])
    td.island_mode = bt.IRM_ALWAYS
    td.clearance, td.min_thickness = 0, nm(0.1)
    td.proto.copper_settings.teardrop.type = bt.TDT_VIA_PAD
    td.proto.copper_settings.connection.zone_connection = bt.ZCS_FULL  # connect_pads yes
    td.proto.id.value = U_Z2
    b.zones.append(td)
    mz = Zone()
    mz.type = bt.ZT_GRAPHICAL
    mz.layers = [bt.BL_F_Mask]
    mz.outline = pwh([(40, 30), (41, 30), (41, 31)])
    b.zones.append(mz)
    ko = Zone()
    ko.type = bt.ZT_RULE_AREA
    ko.layers = [bt.BL_F_Cu]
    ko.name = "KO"
    ko.outline = pwh([(42, 30), (43, 30), (43, 31)])
    b.zones.append(ko)

    for a, c in (((0, 0), (50, 0)), ((50, 0), (50, 40)), ((50, 40), (0, 40)),
                 ((0, 40), (0, 0))):
        b.shapes.append(seg_shape(a, c, bt.BL_Edge_Cuts))
    b.shapes.append(poly_shape(
        [(10, 20), (14, 20), ('arc', (14, 20), (15, 22), (14, 24)), (10, 24), (10, 20)],
        bt.BL_Edge_Cuts, 0.1, False))
    b.shapes.append(poly_shape(
        [(2, 2), (6, 2), ('arc', (6, 2), (7, 4), (6, 6)), (2, 6)],
        bt.BL_F_Cu, 0, True, net_name='GND', uuid=U_GP))
    b.shapes.append(seg_shape((2, 10), (8, 10), bt.BL_B_Cu, width=0.25, uuid=U_GL))
    # Unfilled at stroke 0 -- kipy hands over the LOADED width (0.1); 0 here
    # exercises the builder's guard of the same rule.
    b.shapes.append(rect_shape((10, 30), (12, 32), bt.BL_F_Cu, 0, False))
    b.shapes.append(circle_shape((15, 35), (16, 35), bt.BL_F_Cu, 0.1, True))

    fx, fy, fr = 30, 20, 90
    fp = FootprintInstance()
    fp.position = vec(fx, fy)
    fp.proto.orientation.value_degrees = fr
    fp.layer = bt.BL_F_Cu
    fp.proto.id.value = U_FP
    fp.proto.symbol_sheet_name = '/CH2/'
    fp.proto.symbol_sheet_filename = 'ch.kicad_sch'
    fp.proto.overrides.zone_connection = bt.ZCS_FULL     # (zone_connect 2)
    rf = fp.reference_field
    rf.text.value = 'U1'
    rf.layer = bt.BL_F_SilkS
    rf.visible = True
    rx, ry = g(fx, fy, fr, 0, -2)
    rf.text.position = vec(rx, ry)
    attrs = rf.text.attributes
    attrs.angle = 90
    attrs.size = Vector2.from_xy(nm(1.2), nm(1.0))
    attrs.stroke_width = nm(0.15)
    attrs.horizontal_alignment = 1        # HA_LEFT
    rf.text.attributes = attrs
    fp.reference_field = rf
    nt = fp.proto.definition.net_ties.add()
    nt.pad_number.append('1, 2')
    d = fp.definition
    p1 = make_pad('1', g(fx, fy, fr, 0, 0), bt.PT_SMD, 'SIG',
                  [bt.BL_F_Cu, bt.BL_F_Mask, bt.BL_F_Paste], bt.PSS_CIRCLE, (1, 1))
    p1.pad_to_die_length = nm(1.5)
    p1.proto.pad_to_die_delay.value_as = 12345      # attoseconds: 0.012345 ps
    p1.padstack.proto.zone_settings.zone_connection = bt.ZCS_NONE   # (zone_connect 0)
    d.add_item(p1)
    d.add_item(make_pad('2', g(fx, fy, fr, 2, 0), bt.PT_PTH, 'GND',
                        [bt.BL_F_Cu] + [getattr(bt, f'BL_In{i}_Cu') for i in range(1, 31)]
                        + [bt.BL_B_Cu, bt.BL_F_Mask, bt.BL_B_Mask],
                        bt.PSS_CIRCLE, (1.6, 1.6), drill=0.8))
    d.add_item(make_pad('', g(fx, fy, fr, 4, 0), bt.PT_NPTH, None,
                        [bt.BL_F_Cu] + [getattr(bt, f'BL_In{i}_Cu') for i in range(1, 31)]
                        + [bt.BL_B_Cu, bt.BL_F_Mask, bt.BL_B_Mask],
                        bt.PSS_CIRCLE, (2, 2), drill=2))
    p3 = make_pad('3', g(fx, fy, fr, -3, 0), bt.PT_SMD, 'SIG', [bt.BL_B_Cu, bt.BL_B_Mask],
                  bt.PSS_RECTANGLE, (1, 1),
                  extra_layers=[(bt.BL_In1_Cu, (2, 2)), (bt.BL_B_Cu, (1.5, 0.8))])
    p3.padstack.type = bt.PST_FRONT_INNER_BACK
    # (zone_connect 3) on the primary copper layer only, the padstack-level
    # copy left unset: kipy_pad_zone_connection's fallback.
    p3.padstack.proto.copper_layers[0].zone_settings.zone_connection = bt.ZCS_PTH_THERMAL
    d.add_item(p3)
    for p in d.items:
        if isinstance(p, Pad):
            p.padstack.proto.angle.value_degrees = fr
    d.add_item(seg_shape(g(fx, fy, fr, -1, 3), g(fx, fy, fr, 1, 3), bt.BL_F_Cu, 0.2))
    corners = [g(fx, fy, fr, x, y) for x, y in ((5, -1), (7, -1), (7, 1), (5, 1))]
    for a, c in zip(corners, corners[1:] + corners[:1]):
        d.add_item(seg_shape(a, c, bt.BL_Edge_Cuts))
    fz = Zone()
    fz.layers = [bt.BL_B_Cu]
    fz.net = NETS['GND']
    fz.outline = pwh([(31, 21), (33, 21), (33, 23)])
    fz.island_mode = bt.IRM_ALWAYS
    fz.clearance, fz.min_thickness = nm(0.2), nm(0.25)
    fz.proto.copper_settings.connection.zone_connection = bt.ZCS_THERMAL
    fz.proto.id.value = U_FPZ
    d.add_item(fz)
    d._pack()
    b.fps.append(fp)

    r1 = FootprintInstance()
    r1.position = vec(30, 25)
    r1.layer = bt.BL_F_Cu
    r1.reference_field.text.value = 'R1'
    rd = r1.definition
    rd.add_item(make_pad('1', (30, 25), bt.PT_SMD, 'SIG',
                         [bt.BL_F_Cu, bt.BL_F_Mask, bt.BL_F_Paste],
                         bt.PSS_RECTANGLE, (0.6, 0.6)))
    rd._pack()
    b.fps.append(r1)

    logo = FootprintInstance()
    logo.position = vec(45, 5)
    logo.reference_field.text.value = 'G1'
    ld = logo.definition
    ld.add_item(poly_shape([(45, 5), (46, 5), (46, 6)], bt.BL_F_Cu, 0, True))
    ld._pack()
    b.fps.append(logo)

    grp = Group()
    grp.proto.name = 'sense:U1.1'
    grp.proto.id.value = U_G
    grp.proto.items.add().value = U_T1
    b.groups.append(grp)
    return b


def r6(v):
    return round(v, 6)


def seg_key(s, name_of):
    a, b = (r6(s.start_x), r6(s.start_y)), (r6(s.end_x), r6(s.end_y))
    return (min(a, b), max(a, b), r6(s.width), s.layer, name_of.get(s.net_id, '?'),
            bool(s.graphic), s.owner_ref or '', s.graphic_kind or '',
            bool(s.graphic_filled))


def ring_set(ring):
    return frozenset((r6(x), r6(y)) for x, y in ring)


def run_checks(txt, ipc):
    tname = {nid: n.name for nid, n in txt.nets.items()}
    iname = {nid: n.name for nid, n in ipc.nets.items()}
    tname[0] = iname[0] = ''

    print("A. vias (#1268)")
    tv = {v.uuid: v for v in txt.vias}
    iv = {v.uuid: v for v in ipc.vias}
    check(set(tv) == set(iv) and len(iv) == 5, "every via read, keyed by its uuid")
    for u in sorted(tv):
        a, b = tv[u], iv.get(u)
        if b is None:
            continue
        check((a.via_type, a.layers, r6(a.size), r6(a.drill))
              == (b.via_type, b.layers, r6(b.size), r6(b.drill)),
              f"via {u[:8]}: {a.via_type} {a.layers} == {b.via_type} {b.layers}")
    check(iv[U_V2].layers == ['F.Cu', 'In1.Cu'],
          "a blind via written bottom-first reads top layer first (6552e70f)")
    check(iv[U_V5].layers == ['F.Cu', 'B.Cu'] and iv[U_V5].via_type == 'through',
          "an untyped via reads F.Cu/B.Cu whatever its drill span (db60fe77)")

    print("B. track uuids, sense groups, holds (#1279)")
    tt = sorted(seg_key(s, tname) + (s.uuid, s.held_by)
                for s in txt.segments if not s.graphic)
    it = sorted(seg_key(s, iname) + (s.uuid, s.held_by)
                for s in ipc.segments if not s.graphic)
    check(tt == it, f"tracks agree with uuid and hold ({len(it)} vs {len(tt)})")
    check(sorted(branch_signature(x) for x in txt.sense_branches)
          == sorted(branch_signature(x) for x in ipc.sense_branches)
          and len(ipc.sense_branches) == 1,
          f"resolved sense branches agree: {[branch_signature(x) for x in ipc.sense_branches]}")

    print("C. zones (#1164, holes, arcs, closing vertex, non-copper)")

    def zkey(z, name_of):
        # The island AREA counts only under mode 2 (area): KiCad writes it
        # for that mode alone, so the text reads 0 elsewhere while the live
        # zone carries KiCad's default -- main's pcbnew reader reads it raw
        # too, and compare_pcb_data grades island SEMANTICS, not the field.
        return (z.layer, z.net_name, z.teardrop, tuple((r6(x), r6(y)) for x, y in z.polygon),
                tuple(sorted(ring_set(h) for h in z.holes)), z.priority,
                z.island_removal_mode,
                r6(z.island_area_min) if z.island_removal_mode == 2 else -1.0,
                -1.0 if z.clearance is None else r6(z.clearance),
                -1.0 if z.min_thickness is None else r6(z.min_thickness),
                z.in_footprint, z.uuid,
                # #1329: the pad connection and spoke geometry
                z.pad_connection or '',
                -1.0 if z.thermal_gap is None else r6(z.thermal_gap),
                -1.0 if z.thermal_bridge_width is None else r6(z.thermal_bridge_width))
    tz = sorted(zkey(z, tname) for z in txt.zones)
    iz = sorted(zkey(z, iname) for z in ipc.zones)
    check(tz == iz, f"zones agree ({len(iz)} kipy, {len(tz)} text)")
    if tz != iz:
        for a in sorted(set(tz) ^ set(iz)):
            print("      only in", "text" if a in tz else "kipy", a[:4], len(a[3]))
    check(len(iz) == 4, "4 copper zone entries: the pour on 2 layers, the teardrop, "
                        "the footprint's; the mask zone and the rule area are not pours")
    check(any(z.teardrop for z in ipc.zones), "the teardrop area is flagged (#1164)")
    check(any(len(z.polygon) > 10 for z in ipc.zones),
          "the outline's arc vertex is linearized, not a (0, 0) point (fe367f3f)")
    check(all((0.0, 0.0) not in z.polygon for z in ipc.zones), "no phantom origin vertex")
    pour = [z for z in ipc.zones if z.uuid == U_Z1]
    check(pour and all((z.pad_connection, z.thermal_gap, z.thermal_bridge_width)
                       == ('thermal', 0.5, 0.4) for z in pour)
          and [z.pad_connection for z in ipc.zones if z.teardrop] == ['solid'],
          "zone pad connection and spokes read (#1329): the pour thermal with "
          "its 0.5 gap / 0.4 spokes, the connect_pads-yes teardrop solid")

    print("D. pads (#967 shape/size/layers, #1248 die, zone connection)")
    tp = {p.pad_number: p for p in txt.footprints['U1'].pads}
    ip = {p.pad_number: p for p in ipc.footprints['U1'].pads}
    check(set(tp) == set(ip), f"pad numbers agree {sorted(ip)}")
    for n in sorted(tp):
        a, b = tp[n], ip.get(n)
        if b is None:
            continue
        check(a.shape == b.shape, f"pad {n!r} shape {b.shape} == {a.shape}")
        check(a.pad_type == b.pad_type, f"pad {n!r} kind {b.pad_type} == {a.pad_type}")
        check(set(a.layers) == set(b.layers), f"pad {n!r} layer set {b.layers} == {a.layers}")
        check((r6(a.size_x), r6(a.size_y)) == (r6(b.size_x), r6(b.size_y)),
              f"pad {n!r} size {b.size_x:g}x{b.size_y:g} == {a.size_x:g}x{a.size_y:g}")
        check((r6(a.die_length), r6(a.die_delay)) == (r6(b.die_length), r6(b.die_delay)),
              f"pad {n!r} die {b.die_length:g}mm/{b.die_delay:g}ps")
        check(a.zone_connection == b.zone_connection,
              f"pad {n!r} zone connection {b.zone_connection} == {a.zone_connection}")
    check((ip['1'].zone_connection, ip['2'].zone_connection, ip['3'].zone_connection)
          == ('none', 'solid', 'thru_hole_only'),
          "a pad's own zone_connect wins (pad 1 NONE, read off the padstack; pad 3 "
          "thermal-for-PTH, off its primary copper layer), a pad without one takes "
          "its footprint's (pad 2 solid)")
    check(ip['1'].die_length == 1.5 and abs(ip['1'].die_delay - 0.012345) < 1e-12,
          "pad 1 carries its pad-to-die length and delay")
    check(ip['3'].size_x * ip['3'].size_y < 2.0,
          "a back-only pad of a front/inner/back stack reads its B.Cu entry, not "
          "the inner one it has no copper on (62aace71)")

    print("E. footprint fields (#1211 sheet, #481 label, net ties)")
    ta, ia = txt.footprints['U1'], ipc.footprints['U1']
    check((ta.sheet_name, ta.sheet_file) == (ia.sheet_name, ia.sheet_file) == ('/CH2/', 'ch.kicad_sch'),
          f"sheet name/file {ia.sheet_name!r} {ia.sheet_file!r}")
    # The shipped two-path diff sees the sheet fields: clean on the two reads,
    # and a sheet name bent on one side is reported (mutate_1211's comparator
    # row is killed here).
    sheet_diffs = [d for d in kp.compare_pcb_data(ipc, txt) if '_sheet' in d or 'sheet_' in d]
    check(not sheet_diffs, f"compare_pcb_data reports no sheet difference {sheet_diffs[:2]}")
    saved = ia.sheet_name
    ia.sheet_name = '/OTHER/'
    try:
        bent = [d for d in kp.compare_pcb_data(ipc, txt) if 'sheet_name' in d]
    finally:
        ia.sheet_name = saved
    check(len(bent) == 1 and 'U1' in bent[0],
          f"a sheet name changed on one side is reported: {bent}")
    check(sorted(map(sorted, ta.net_tie_groups)) == sorted(map(sorted, ia.net_tie_groups)) == [['1', '2']],
          f"net-tie groups {ia.net_tie_groups}")
    check((ta.zone_connection, ia.zone_connection) == ('solid', 'solid')
          and txt.footprints['R1'].zone_connection is None
          and ipc.footprints['R1'].zone_connection is None,
          "the footprint zone_connect override reads (U1 solid, R1 none set)")
    zc_diffs = [d for d in kp.compare_pcb_data(ipc, txt) if 'zone_connection' in d]
    check(not zc_diffs, f"compare_pcb_data reports no zone_connection difference {zc_diffs[:2]}")
    pad1 = next(p for p in ia.pads if p.pad_number == '1')
    pad1.zone_connection = None
    try:
        bent = [d for d in kp.compare_pcb_data(ipc, txt) if 'zone_connection' in d]
    finally:
        pad1.zone_connection = 'none'
    check(len(bent) == 1 and 'U1:1' in bent[0],
          f"a pad's zone connection dropped on one side is reported: {bent}")
    tl, il = ta.ref_label, ia.ref_label
    check(il is not None, "the Reference label is read (it never was on IPC)")
    if tl is not None and il is not None:
        check((r6(tl.at_x), r6(tl.at_y), r6(tl.rotation), r6(tl.size_h), r6(tl.size_w),
               r6(tl.thickness), tl.layer, tl.justify, tl.hidden)
              == (r6(il.at_x), r6(il.at_y), r6(il.rotation), r6(il.size_h), r6(il.size_w),
                  r6(il.thickness), il.layer, il.justify, il.hidden),
              f"label {il} == text")

    print("F. Edge.Cuts (#304, 6febcfad) and outline ownership (#829)")
    check(sorted(map(ring_set, txt.board_info.board_outlines))
          == sorted(map(ring_set, ipc.board_info.board_outlines))
          and len(ipc.board_info.board_outlines) == 1,
          "one outer ring, the line-drawn outline (the old kipy reader read none)")
    check(sorted(map(ring_set, txt.board_info.board_cutouts))
          == sorted(map(ring_set, ipc.board_info.board_cutouts))
          and len(ipc.board_info.board_cutouts) == 2,
          f"two cut-outs agree: the arc gr_poly and the footprint window "
          f"({len(ipc.board_info.board_cutouts)})")
    check(tuple(map(r6, txt.board_info.board_bounds or ()))
          == tuple(map(r6, ipc.board_info.board_bounds or ())),
          f"bounds {ipc.board_info.board_bounds}")
    check((ta.owns_edge_cuts, ta.owns_board_outline) == (ia.owns_edge_cuts, ia.owns_board_outline)
          == (True, False), "U1 draws Edge.Cuts, a carried window, so it stays movable")

    print("G. copper graphics (#337, #908, 6925b417)")
    tg = sorted(seg_key(s, tname) for s in txt.segments if s.graphic)
    ig = sorted(seg_key(s, iname) for s in ipc.segments if s.graphic)
    check(tg == ig, f"graphic copper agrees ({len(ig)} kipy, {len(tg)} text segments)")
    if tg != ig:
        for a in sorted(set(tg) ^ set(ig))[:8]:
            print("      only in", "text" if a in tg else "kipy", a)
    check(not any(s.owner_ref == 'G1' for s in ipc.segments),
          "the pad-less logo's copper is not modelled")
    check(any(s.owner_ref == 'U1' for s in ipc.segments if s.graphic),
          "the pad-bearing footprint's copper is, owned by its key")
    check(any(s.graphic and s.graphic_kind == 'rect' and r6(s.width) == 0.1
              for s in ipc.segments),
          "an unfilled zero-width rect is stroked at 0.1 mm (pcbnew's loader)")


def test_via_type_round_trip():
    """The adapter writes a type, the builder reads it back."""
    writer = getattr(adapter, 'kipy_via_type_value', None)
    if writer is None:
        check(False, "kicad_ipc_adapter.kipy_via_type_value exists (the writer)")
        return
    for t in ('through', 'blind', 'buried', 'micro'):
        v = Via()
        val = writer(t)
        if val is not None:
            v.type = val
        check(kp.kipy_via_type(v) == t, f"{t!r} written and read back")
    check(kp.kipy_via_type(object()) == 'through', "an unreadable via reads 'through'")


def test_sense_groups_reader(b):
    got = kp.kipy_sense_groups(b)
    want = [(n, u, list(m)) for n, u, m in sense_groups_from_text(BOARD)]
    check(got == want, f"kipy_sense_groups {got} == sense_groups_from_text")

    class _NoGroups:
        def get_groups(self):
            raise RuntimeError("old kipy")
    check(kp.kipy_sense_groups(_NoGroups()) == [], "an unreadable group list reads []")


def test_dialog_resync(b, dialog):
    """The IPC dialog keeps ONE PCBData and re-syncs it from a fresh build
    before each step. `kipy_resolve_sense_branches` re-resolves the groups on
    the dialog's own PCBData; adopting the fresh build's branches instead
    leaves the dialog's pads tagged by the LAST resolution."""
    def r1_pad(pcb):
        return pcb.footprints['R1'].pads[0]
    check(r1_pad(dialog).sense == 'sense:U1.1',
          "the branch's far-end pad is tagged a sense pad at the first build")
    saved = list(b.groups)
    b.groups[:] = []                      # the group is deleted in the session
    try:
        orig = adapter.get_board_full_path
        adapter.get_board_full_path = lambda *a, **k: None
        try:
            fresh = kp.build_pcb_data_from_board(b)
        finally:
            adapter.get_board_full_path = orig
        check(fresh.sense_branches == [] and not r1_pad(fresh).sense,
              "a fresh build sees no group")
        dialog.segments, dialog.vias = fresh.segments, fresh.vias
        adopted = fresh.sense_branches
        check(adopted == [] and r1_pad(dialog).sense == 'sense:U1.1',
              "negative control: adopting the fresh build's branches leaves the "
              "dialog's pad tagged by the deleted group")
        kp.kipy_resolve_sense_branches(dialog, b)
        check(dialog.sense_branches == [] and not r1_pad(dialog).sense,
              "kipy_resolve_sense_branches clears the dialog's stale tag")
    finally:
        b.groups[:] = saved
    kp.kipy_resolve_sense_branches(dialog, b)
    check(len(dialog.sense_branches) == 1 and r1_pad(dialog).sense == 'sense:U1.1',
          "and a group drawn again counts at the next re-sync")


CUSTOM_BOARD = '''(kicad_pcb (version 20260206) (generator "pcbnew")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (1 "F.Mask" user) (25 "Edge.Cuts" user))
  (net 0 "") (net 1 "D")
  (footprint "lib:Q" (layer "F.Cu") (at 10 10)
    (property "Reference" "Q1" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd custom (at 0 0) (size 1 1) (layers "F.Cu" "F.Mask") (net 1 "D")
      (options (clearance outline) (anchor rect))
      (primitives (gr_poly (pts (xy 0 -0.5) (xy 4 -0.5) (xy 4 0.5) (xy 0 0.5))
        (width 0) (fill yes))))
    (pad "2" smd custom (at 6 0) (size 1 1) (layers "F.Cu" "F.Mask") (net 1 "D")
      (options (clearance outline) (anchor rect))
      (primitives (gr_poly (pts (xy -2 -0.5) (xy 2 -0.5) (xy 2 0.5) (xy -2 0.5))
        (width 0) (fill yes)))))
)
'''


def test_custom_pad_copper_box(tmp):
    """A custom pad's copper drawn to ONE side of its anchor carries
    Pad.copper_box (main's pcbnew builder read it off the effective shape's
    box; the kipy builder reads the rendered polygon, `_size_custom_pad_from_
    copper`), and a pad drawn about its anchor carries none. The kipy pad's
    polygon is the union outline KiCad renders for that copper."""
    import copy
    path = os.path.join(tmp, 'custom.kicad_pcb')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(CUSTOM_BOARD)
    txt = {p.pad_number: p for p in kp.parse_kicad_pcb(path).footprints['Q1'].pads}
    rendered = {'1': [(9.5, 9.5), (14, 9.5), (14, 10.5), (9.5, 10.5)],
                '2': [(14, 9.5), (18, 9.5), (18, 10.5), (14, 10.5)]}
    prim = {'1': [(0, -0.5), (4, -0.5), (4, 0.5), (0, 0.5)],
            '2': [(-2, -0.5), (2, -0.5), (2, 0.5), (-2, 0.5)]}
    for n in ('1', '2'):
        kpad = Pad()
        cl = kpad.padstack.proto.copper_layers[0]     # a fresh Pad has one
        cl.layer, cl.shape = bt.BL_F_Cu, bt.PSS_CUSTOM
        cl.size.x_nm = cl.size.y_nm = nm(1)
        cl.custom_shapes.add().CopyFrom(poly_shape(prim[n], bt.BL_F_Cu, 0, True).proto)
        got = copy.copy(txt[n])
        got.copper_box, got.size_x, got.size_y = None, 1.0, 1.0
        got.anchor_size = (1.0, 1.0)
        got.polygons = [rendered[n]]
        kp._size_custom_pad_from_copper(got, kpad)
        want = txt[n].copper_box
        check((None if want is None else tuple(map(r6, want)))
              == (None if got.copper_box is None else tuple(map(r6, got.copper_box))),
              f"custom pad {n}: copper_box {got.copper_box} == text {want}")
        check((r6(got.size_x), r6(got.size_y)) == (r6(txt[n].size_x), r6(txt[n].size_y)),
              f"custom pad {n}: size {got.size_x:g}x{got.size_y:g} == text")
    check(txt['1'].copper_box is not None and txt['2'].copper_box is None,
          "the fixture is the case: pad 1 offset, pad 2 drawn about its anchor")


ROUND_BOARD = '''(kicad_pcb (version 20260206) (generator "pcbnew")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "") (net 1 "GND") (net 2 "SIG")
  (footprint "lib:C" (layer "F.Cu") (at 10 10)
    (property "Reference" "C1" (at 0 0) (layer "F.Cu"))
    (pad "1" smd roundrect (at 0 0 30) (size 2 1) (layers "F.Cu")
      (roundrect_rratio 0.25) (chamfer_ratio 0.2) (chamfer top_left bottom_right)
      (net 2 "SIG")))
  (footprint "lib:W" (layer "F.Cu") (at 60 20 30)
    (property "Reference" "W1" (at 0 0) (layer "F.Cu"))
    (pad "1" smd rect (at 0 0 30) (size 1 1) (layers "F.Cu") (net 2 "SIG"))
    (fp_rect (start 2 -1) (end 4 1) (stroke (width 0.1) (type solid)) (fill none)
      (layer "Edge.Cuts")))
  (gr_line (start 0 0) (end 50 0) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts"))
  (gr_line (start 50 0) (end 50 40) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts"))
  (gr_line (start 50 40) (end 0 40) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts"))
  (gr_line (start 0 40) (end 0 0) (stroke (width 0.1) (type solid)) (layer "Edge.Cuts"))
  (gr_circle (center 25 20) (end 28 20) (stroke (width 0.1) (type solid)) (fill none)
    (layer "Edge.Cuts"))
)
'''


def build_round_board():
    b = FakeBoard()
    for a, c in (((0, 0), (50, 0)), ((50, 0), (50, 40)), ((50, 40), (0, 40)),
                 ((0, 40), (0, 0))):
        b.shapes.append(seg_shape(a, c, bt.BL_Edge_Cuts))
    b.shapes.append(circle_shape((25, 20), (28, 20), bt.BL_Edge_Cuts, 0.1, False))

    c1 = FootprintInstance()
    c1.position = vec(10, 10)
    c1.layer = bt.BL_F_Cu
    c1.reference_field.text.value = 'C1'
    p = make_pad('1', (10, 10), bt.PT_SMD, 'SIG', [bt.BL_F_Cu],
                 bt.PSS_CHAMFEREDRECT, (2, 1))
    cl = p.padstack.proto.copper_layers[0]
    cl.corner_rounding_ratio = 0.25
    cl.chamfer_ratio = 0.2
    cl.chamfered_corners.top_left = True
    cl.chamfered_corners.bottom_right = True
    p.padstack.proto.angle.value_degrees = 30
    c1.definition.add_item(p)
    c1.definition._pack()
    b.fps.append(c1)

    fx, fy, fr = 60, 20, 30
    w1 = FootprintInstance()
    w1.position = vec(fx, fy)
    w1.proto.orientation.value_degrees = fr
    w1.layer = bt.BL_F_Cu
    w1.reference_field.text.value = 'W1'
    wp = make_pad('1', g(fx, fy, fr, 0, 0), bt.PT_SMD, 'SIG', [bt.BL_F_Cu],
                  bt.PSS_RECTANGLE, (1, 1))
    wp.padstack.proto.angle.value_degrees = fr
    w1.definition.add_item(wp)
    corners = [g(fx, fy, fr, x, y) for x, y in ((2, -1), (4, -1), (4, 1), (2, 1))]
    for a, c in zip(corners, corners[1:] + corners[:1]):
        w1.definition.add_item(seg_shape(a, c, bt.BL_Edge_Cuts))
    w1.definition._pack()
    b.fps.append(w1)
    return b


def test_main_round_ports(tmp):
    path = os.path.join(tmp, 'round.kicad_pcb')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(ROUND_BOARD)
    txt = kp.parse_kicad_pcb(path)
    orig = adapter.get_board_full_path
    adapter.get_board_full_path = lambda *a, **k: path
    try:
        ipc = kp.build_pcb_data_from_board(build_round_board())
    finally:
        adapter.get_board_full_path = orig

    tp = txt.footprints['C1'].pads[0]
    ip = ipc.footprints['C1'].pads[0]
    check(bool(tp.polygons), "#1450: the text parse gives the chamfered pad an "
          "outline (the arm tests something)")

    def rings(pad):
        return [[(r6(x), r6(y)) for x, y in ring] for ring in (pad.polygons or [])]
    check(rings(ip) == rings(tp),
          f"#1450: the kipy pad's outline is the text parser's "
          f"({len((ip.polygons or [[]])[0])} vs {len(tp.polygons[0])} vertices)")
    check(ip.shape == tp.shape == 'roundrect',
          f"#1450: shape {ip.shape!r} == text {tp.shape!r}")

    tw, iw = txt.footprints['W1'], ipc.footprints['W1']
    check(tw.owns_edge_cuts and iw.owns_edge_cuts,
          "#1409: W1 draws Edge.Cuts on both paths")
    check(tw.owns_board_outline is False,
          "#1409: the text parse carries W1's window off the board "
          "(the arm tests something)")
    check(iw.owns_board_outline == tw.owns_board_outline,
          f"#1409: owns_board_outline kipy={iw.owns_board_outline} == "
          f"text={tw.owns_board_outline}")

    def cut_rings(pcb):
        return sorted(len(r) for r in (pcb.board_info.board_cutouts or []))
    check(cut_rings(txt) and cut_rings(ipc) == cut_rings(txt),
          f"#1404: the round cut-out has the text parser's vertex count "
          f"(kipy {cut_rings(ipc)}, text {cut_rings(txt)})")
    tr = next((r for r in txt.board_info.board_cutouts or [] if len(r) > 64), None)
    ir = next((r for r in ipc.board_info.board_cutouts or [] if len(r) > 64), None)
    check(tr is not None and ir is not None and ring_set(ir) == ring_set(tr),
          "#1404: at EDGE_CHORD_EPS (more than the old 64 chords), vertex for "
          "vertex")


def test_negative_controls(b):
    """The old readers' failures, reproduced, so a passing check above is the
    port and not the fixture."""
    shape = b.shapes[0]
    check(getattr(shape, 'shape_type', None) is None,
          "a kipy shape has no `shape_type` (why the old Edge.Cuts reader read none)")
    pad_shape = b.fps[0].definition.pads[0].padstack.copper_layers[0].shape
    check(not hasattr(pad_shape, 'name') and kp._pad_shape_label(str(pad_shape)) == 'rect',
          "the PadStackShape is a bare int: read by str() every pad was 'rect'")
    arc_node = [n for n in b.zones[0].outline.outline.nodes if n.has_arc][0]
    check((arc_node.point.x, arc_node.point.y) == (0, 0),
          "an arc node read as a point is (0, 0) (why outlines carried the origin)")


def main():
    tmp = tempfile.mkdtemp(prefix='t1279kipy_')
    try:
        path = os.path.join(tmp, 'fixture.kicad_pcb')
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(BOARD)
        txt = kp.parse_kicad_pcb(path)
        b = build_kipy_board()
        orig = adapter.get_board_full_path
        adapter.get_board_full_path = lambda *a, **k: path
        try:
            ipc = kp.build_pcb_data_from_board(b)
        finally:
            adapter.get_board_full_path = orig
        run_checks(txt, ipc)
        print("H. via type writer round trip, sense-group reader, negative controls")
        test_via_type_round_trip()
        test_sense_groups_reader(b)
        test_negative_controls(b)
        print("I. the dialog re-sync's sense groups (kipy_resolve_sense_branches)")
        test_dialog_resync(b, ipc)
        print("J. a custom pad's copper box (Pad.copper_box)")
        test_custom_pad_copper_box(tmp)
        print("K. chamfered pads (#1450), carried windows (#1409), edge arcs (#1404)")
        test_main_round_ports(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f"\n{len(FAILS)} check(s) FAILED")
        return 1
    print("\nPASS  kipy builder parity with the text parser (#1268 #1279 #1164 "
          "#967 #1248 #1211 #481 #304 #829 #337 #908 #1329, zone_connect, "
          "copper_box, #1450 #1409 #1404)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
