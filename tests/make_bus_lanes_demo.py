"""Builds tests/fixtures/bus_lanes/demo.kicad_pcb, the laid-lanes fixture (tests/test_bus_lanes_e2e.py), under KiCad's
Python (pcbnew). The board is tracked; this script says how it was made and remakes it:

    python3 tests/make_bus_lanes_demo.py

A 4-layer 60 x 30 mm board. Six nets A0..A5, an ordered bus: each with a far end (a 1 x 1 mm SMD pad) alternating
between the top and the bottom edge, x 8 .. 28 (tributaries from both sides), and an outlet pad in a column at x 52,
1.5 mm apart in the bus's order (A0 at the top). A pad of another net (OBST) sits on A3's lane where it would join, and
a pad of another MEMBER (A5's second pad) just past it, so A3's drop moves downstream of both: the drop is checked
against the other members' copper too. The bus's centreline, drawn on User.2: (4, 15) -> (40, 15) -> (48, 15); laid with a fan of
1.5 mm, the lanes meet the outlet's rows.
"""
import os
import pcbnew

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, 'tests', 'fixtures', 'bus_lanes', 'demo.kicad_pcb')
mm = pcbnew.FromMM


def P(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


def main():
    b = pcbnew.BOARD()
    b.SetCopperLayerCount(4)
    ds = b.GetDesignSettings()
    ds.SetCopperLayerCount(4)
    for (x0, y0), (x1, y1) in (((0, 0), (60, 0)), ((60, 0), (60, 30)), ((60, 30), (0, 30)), ((0, 30), (0, 0))):
        s = pcbnew.PCB_SHAPE(b)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT); s.SetStart(P(x0, y0)); s.SetEnd(P(x1, y1))
        s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(mm(0.1)); b.Add(s)
    nets = {}

    def net(name):
        if name not in nets:
            b.Add(pcbnew.NETINFO_ITEM(b, name, len(nets) + 1))
            nets[name] = b.FindNet(name)
        return nets[name]

    def pad(ref, x, y, name, w=1.0, h=1.0):
        f = pcbnew.FOOTPRINT(b)
        f.SetReference(ref); f.SetPosition(P(x, y)); b.Add(f)
        p = pcbnew.PAD(f)
        p.SetNumber('1'); p.SetShape(pcbnew.PAD_SHAPE_RECT); p.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
        ls = pcbnew.LSET(); ls.AddLayer(pcbnew.F_Cu); p.SetLayerSet(ls)
        p.SetSize(pcbnew.VECTOR2I(mm(w), mm(h))); p.SetPosition(P(x, y)); p.SetNet(net(name))
        f.Add(p)

    for i in range(6):
        name = f'A{i}'
        pad(f'TAP{i}', 8 + 4 * i, 2.0 if i % 2 == 0 else 28.0, name)       # far ends, both edges
        pad(f'OUT{i}', 52.0, 15.0 - 3.75 + 1.5 * i, name, w=1.5, h=0.8)    # the outlet, in order
    pad('OB1', 20.4, 15.0, 'OBST', w=0.8, h=0.8)                           # on A3's lane, where it would join
    pad('A5X', 21.6, 15.33, 'A5', w=0.8, h=0.8)                            # another member's, just past it
    pad('OB2', 30.0, 2.0, 'OBST', w=0.8, h=0.8)
    for (x0, y0), (x1, y1) in (((4, 15), (40, 15)), ((40, 15), (48, 15))):
        s = pcbnew.PCB_SHAPE(b)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT); s.SetStart(P(x0, y0)); s.SetEnd(P(x1, y1))
        s.SetLayer(pcbnew.User_2); s.SetWidth(mm(0.1)); b.Add(s)
    b.Save(OUT)
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
