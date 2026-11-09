"""Build tests/fixtures/1643/edge_gap.kicad_pcb (needs KiCad's pcbnew).

usage: make_fixture.py OUT.kicad_pcb ZONE_CLEARANCE_MM EDGE_LINE_WIDTH_MM
       (the fixture: OUT 0.2 0.1; pcbnew also writes a .kicad_pro and a
       .kicad_prl beside it, which are not kept -- each case writes its own)

40x30 mm board, a GND pour on F.Cu and on B.Cu drawn 1 mm past the outline
on every side, one GND through-hole pad at (6, 15), islands kept.
tests/test_1643_zone_edge_gap.py redraws the outline (its four Edge.Cuts
lines) as two halves joined by an isthmus, whose fill splits exactly when
the isthmus is narrower than twice the edge gap plus min_thickness.
"""
import sys, pcbnew
from pcbnew import FromMM as mm, VECTOR2I as V
out, zc, ew = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
b = pcbnew.BOARD()
W, H = 40.0, 30.0
for (x1, y1, x2, y2) in [(0, 0, W, 0), (W, 0, W, H), (W, H, 0, H), (0, H, 0, 0)]:
    s = pcbnew.PCB_SHAPE(b); s.SetShape(pcbnew.SHAPE_T_SEGMENT); s.SetLayer(pcbnew.Edge_Cuts)
    s.SetStart(V(mm(x1), mm(y1))); s.SetEnd(V(mm(x2), mm(y2))); s.SetWidth(mm(ew)); b.Add(s)
gnd = pcbnew.NETINFO_ITEM(b, "GND"); b.Add(gnd)
f = pcbnew.FOOTPRINT(b); f.SetReference("J1"); f.SetPosition(V(mm(6), mm(15)))
p = pcbnew.PAD(f); p.SetNumber("1"); p.SetShape(pcbnew.PAD_SHAPE_CIRCLE); p.SetAttribute(pcbnew.PAD_ATTRIB_PTH)
p.SetLayerSet(pcbnew.PAD.PTHMask()); p.SetSize(V(mm(1.6), mm(1.6))); p.SetDrillSize(V(mm(0.8), mm(0.8)))
p.SetPosition(V(mm(6), mm(15))); p.SetNet(gnd); f.Add(p); b.Add(f)
for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
    z = pcbnew.ZONE(b); z.SetLayer(lay); z.SetNet(gnd)
    z.SetLocalClearance(mm(zc)); z.SetMinThickness(mm(0.25))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_NEVER)
    ol = z.Outline(); ol.NewOutline()
    for x, y in [(-1, -1), (W + 1, -1), (W + 1, H + 1), (-1, H + 1)]:
        ol.Append(mm(x), mm(y))
    b.Add(z)
b.Save(out)
