#!/usr/bin/env python3
"""Build tests/fixtures/1449/edge_neck.kicad_pcb + .kicad_pro (needs KiCad's pcbnew).

40x30 mm, 2 layers, one F.Cu GND pour, split by a 1 mm pad of net WALL except
for a neck of width GAP between the wall's end and the bottom board edge. With
the project's min_copper_edge_clearance at EDGE the neck is GAP - EDGE - 0.2
(the zone clearance to the wall) wide: 0.35 mm at the fixture's 0.05 edge,
which survives the 0.25 min thickness, and 0.20 mm at the 0.2 fab-edge pin,
which does not. From the #1449 report.

usage: make_fixture.py OUTDIR [GAP_MM] [EDGE_MM]
"""
import sys, json, os, pcbnew
from pcbnew import FromMM as mm, VECTOR2I as V
out = sys.argv[1]; gap = float(sys.argv[2]) if len(sys.argv) > 2 else 0.6; edge = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
os.makedirs(out, exist_ok=True)
b = pcbnew.BOARD()
ds = b.GetDesignSettings()
ds.m_CopperEdgeClearance = mm(edge)
W, H = 40.0, 30.0
for (x1, y1, x2, y2) in [(0,0,W,0),(W,0,W,H),(W,H,0,H),(0,H,0,0)]:
    s = pcbnew.PCB_SHAPE(b); s.SetShape(pcbnew.SHAPE_T_SEGMENT); s.SetLayer(pcbnew.Edge_Cuts)
    s.SetStart(V(mm(x1),mm(y1))); s.SetEnd(V(mm(x2),mm(y2))); s.SetWidth(mm(0.1)); b.Add(s)
nets = {}
def net(n):
    if n not in nets:
        ni = pcbnew.NETINFO_ITEM(b, n); b.Add(ni); nets[n] = ni
    return nets[n]
def fp(ref, x, y, pads):
    f = pcbnew.FOOTPRINT(b); f.SetReference(ref); f.SetPosition(V(mm(x),mm(y)))
    for num, dx, dy, w, h, n in pads:
        p = pcbnew.PAD(f); p.SetNumber(num); p.SetShape(pcbnew.PAD_SHAPE_RECT); p.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
        p.SetLayerSet(pcbnew.LSET().AddLayer(pcbnew.F_Cu)); p.SetSize(V(mm(w),mm(h))); p.SetPosition(V(mm(x+dx),mm(y+dy)))
        p.SetNet(net(n)); f.Add(p)
    b.Add(f)
fp("J1", 8, 15, [("1", 0, -3, 1.0, 1.0, "GND")])
fp("J2", 32, 15, [("1", 0, 3, 1.0, 1.0, "GND")])
fp("R1", 8, 8, [("1", 0, 0, 1.0, 1.0, "SIG"), ])
fp("R2", 8, 22, [("1", 0, 0, 1.0, 1.0, "SIG")])
# wall: one long pad of net WALL at x=20, from y=0.3 down to (H - gap)
y0, y1 = 0.3, H - gap
fp("W1", 20, (y0+y1)/2, [("1", 0, 0, 1.0, y1-y0, "WALL")])
z = pcbnew.ZONE(b); z.SetLayer(pcbnew.F_Cu); z.SetNet(net("GND"))
z.SetLocalClearance(mm(0.2)); z.SetMinThickness(mm(0.25))
z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_NEVER)  # keep islands, so fills do not hide the split
ol = z.Outline(); ol.NewOutline()
for x, y in [(0,0),(W,0),(W,H),(0,H)]: ol.Append(mm(x), mm(y))
b.Add(z)
b.BuildConnectivity()
b.Save(os.path.join(out, "edge_neck.kicad_pcb"))
pro = {"meta": {"filename": "edge_neck.kicad_pro", "version": 3}, "board": {"design_settings": {"rules": {"min_copper_edge_clearance": edge}}}, "net_settings": {"classes": [{"name":"Default","clearance":0.2,"track_width":0.2,"via_diameter":0.6,"via_drill":0.3}], "meta":{"version":4}}}
json.dump(pro, open(os.path.join(out, "edge_neck.kicad_pro"), "w"), indent=2)
