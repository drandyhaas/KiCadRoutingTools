#!/usr/bin/env python3
"""nudge_grazing_microshift clears grazes necking/re-bending can't (issue #276).

Three shapes, mirroring the real stress-corpus defects:
  * VERTEX: a necked terminal joint whose ENDPOINT sits 8-16um inside the
    required clearance of a foreign pad (ottercast C58.1). The octolinear
    re-bend keeps anchors fixed, so only sliding the vertex itself can fix it.
  * BOW: a long leg passing sub-clearance of a foreign via mid-segment, its
    near end anchored by its own via (butterstick CATG vs SDMMC0.DAT1).
  * CAP: a graze needing more than max_shift must be left alone -- the pass is
    a micrometre-scale touch-up, never a wild move.

    python3 tests/test_nudge_microshift.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, Net, PCBData, BoardInfo, Segment, Via
from pcb_modification import nudge_grazing_microshift
from check_connected import check_net_connectivity


def _pad(ref, x, y, sx, sy, net_id, net_name, layer='F.Cu'):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=sx, size_y=sy, shape='rect',
               layers=[layer], net_id=net_id, net_name=net_name, drill=0)


def _board(pads_by_net, segments, vias):
    return PCBData(footprints={}, nets={1: Net(1, '/SIG'), 2: Net(2, '/OTHER')},
                   segments=segments, vias=vias,
                   board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                                        board_bounds=(0.0, 0.0, 200.0, 200.0)),
                   pads_by_net=pads_by_net)


def _seg_pad_dist(s, pad):
    """Min distance from segment centreline to the pad's axis rect."""
    best = 1e9
    n = 200
    for i in range(n + 1):
        t = i / n
        px = s.start_x + (s.end_x - s.start_x) * t
        py = s.start_y + (s.end_y - s.start_y) * t
        dx = max(abs(px - pad.global_x) - pad.size_x / 2, 0.0)
        dy = max(abs(py - pad.global_y) - pad.size_y / 2, 0.0)
        best = min(best, math.hypot(dx, dy))
    return best


def _seg_via_dist(s, via):
    best = 1e9
    n = 200
    for i in range(n + 1):
        t = i / n
        px = s.start_x + (s.end_x - s.start_x) * t
        py = s.start_y + (s.end_y - s.start_y) * t
        best = min(best, math.hypot(px - via.x, py - via.y) - via.size / 2)
    return best


def _comps(pcb, nid):
    segs = [s for s in pcb.segments if s.net_id == nid]
    vias = [v for v in pcb.vias if v.net_id == nid]
    r = check_net_connectivity(nid, segs, vias, pcb.pads_by_net.get(nid, []), [])
    return r.get('num_components')


def _vertex_fixture(pad_x=115.47):
    """Ottercast shape: pad-escape stub joined by copper overlap to a necked
    terminal seg whose start vertex grazes the foreign pad at `pad_x`."""
    own = _pad('U6', 115.8, 90.6, 0.25, 0.9, 1, '/SIG')
    foreign = _pad('C58', pad_x, 89.60, 0.5, 0.6, 2, '/OTHER')
    segs = [
        Segment(start_x=115.8, start_y=90.6, end_x=115.8, end_y=90.05,
                width=0.1, layer='F.Cu', net_id=1),
        Segment(start_x=115.8, start_y=90.0, end_x=115.9, end_y=90.1,
                width=0.0889, layer='F.Cu', net_id=1),   # grazing terminal joint
        Segment(start_x=115.9, start_y=90.1, end_x=116.0, end_y=90.0,
                width=0.127, layer='F.Cu', net_id=1),
        Segment(start_x=116.0, start_y=90.0, end_x=116.0, end_y=89.2,
                width=0.127, layer='F.Cu', net_id=1),
    ]
    pcb = _board({1: [own], 2: [foreign]}, segs, [])
    return pcb, foreign


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)
        print(("  PASS " if cond else "  FAIL ") + name)

    CLR = 0.1

    # --- VERTEX: terminal-joint endpoint graze (ottercast C58.1 shape) -------
    pcb, foreign = _vertex_fixture()
    grazing = [s for s in pcb.segments if s.width == 0.0889][0]
    short = (CLR + grazing.width / 2) - _seg_pad_dist(grazing, foreign)
    check("vertex fixture actually grazes (~16um)", 0.005 < short < 0.025)
    comps0 = _comps(pcb, 1)
    n_ch, n_nets, removed, added = nudge_grazing_microshift(
        [], pcb, {1}, clearance=CLR, max_shift=0.025)
    check("vertex: pass fired", n_ch > 0 and n_nets == 1)
    worst = min(_seg_pad_dist(s, foreign) - (CLR + s.width / 2 - 1e-4)
                for s in pcb.segments if s.net_id == 1)
    check("vertex: graze cleared", worst >= 0)
    check("vertex: connectivity preserved", _comps(pcb, 1) == comps0)

    # audit: each added endpoint is within max_shift of SOME original endpoint
    originals = [(115.8, 90.6), (115.8, 90.05), (115.8, 90.0), (115.9, 90.1),
                 (116.0, 90.0), (116.0, 89.2)]
    ok = True
    for a in added:
        for (ex, ey) in ((a.start_x, a.start_y), (a.end_x, a.end_y)):
            if min(math.hypot(ex - ox, ey - oy) for ox, oy in originals) > 0.025 + 1e-6:
                ok = False
    check("vertex: no endpoint moved farther than max_shift", ok)

    # --- BOW: mid-segment graze vs a foreign via (butterstick CATG shape) ----
    CLR2 = 0.0889
    own_via = Via(x=167.3, y=84.2, size=0.45, drill=0.2,
                  layers=['F.Cu', 'B.Cu'], net_id=1)
    foreign_via = Via(x=167.65, y=85.2, size=0.45, drill=0.2,
                      layers=['F.Cu', 'B.Cu'], net_id=2)
    segs = [
        Segment(start_x=167.3, start_y=84.2, end_x=167.3, end_y=90.8,
                width=0.0889, layer='B.Cu', net_id=1),   # grazes foreign via at y=85.2
        Segment(start_x=167.3, start_y=90.8, end_x=168.0, end_y=91.5,
                width=0.0889, layer='B.Cu', net_id=1),
    ]
    pcb2 = _board({1: [], 2: []}, segs, [own_via, foreign_via])
    g2 = segs[0]
    short2 = (CLR2 + g2.width / 2) - _seg_via_dist(g2, foreign_via)
    check("bow fixture actually grazes (~8um)", 0.002 < short2 < 0.02)
    comps0 = _comps(pcb2, 1)
    n_ch2, n_nets2, removed2, added2 = nudge_grazing_microshift(
        [], pcb2, {1}, clearance=CLR2, max_shift=0.025)
    check("bow: pass fired", n_ch2 > 0)
    worst2 = min(_seg_via_dist(s, foreign_via) - (CLR2 + s.width / 2 - 1e-4)
                 for s in pcb2.segments if s.net_id == 1)
    check("bow: graze cleared", worst2 >= 0)
    check("bow: connectivity preserved", _comps(pcb2, 1) == comps0)
    check("bow: own via endpoint did not move",
          any(abs(s.start_x - 167.3) < 1e-6 and abs(s.start_y - 84.2) < 1e-6
              for s in pcb2.segments if s.net_id == 1))

    # --- HOLE: a track grazing a foreign NPTH drill hole (urti J3 shape, #308) --
    # NPTH mounting holes carry no copper, so the pad/seg/via terms never see
    # them; a track crossing one is graded at the higher NPTH-to-track floor.
    CLR3 = 0.09
    NPTH_CLR = 0.20  # max(CLR3, NPTH_TO_TRACK_CLEARANCE)
    HR = 0.25        # drill 0.5 -> radius 0.25
    W = 0.0889
    OVERLAP = 0.017  # urti's measured graze
    npth = _pad('J3', 100.0, 100.0, 0.5, 0.5, 2, '/OTHER')
    npth.drill = 0.5
    npth.pad_type = 'np_thru_hole'
    # Own GND net: two pads with a track between them, grazing the hole mid-span.
    gy = 100.0 - (HR + NPTH_CLR + W / 2 - OVERLAP)
    own_a = _pad('U1', 98.5, gy, 0.4, 0.4, 1, '/SIG')
    own_b = _pad('U2', 101.5, gy, 0.4, 0.4, 1, '/SIG')
    segsH = [Segment(start_x=98.5, start_y=gy, end_x=101.5, end_y=gy,
                     width=W, layer='F.Cu', net_id=1)]
    pcbH = _board({1: [own_a, own_b], 2: [npth]}, segsH, [])
    from single_ended_routing import _seg_foreign_hole_dist
    hole_d = _seg_foreign_hole_dist(pcbH, 1, 98.5, gy, 101.5, gy)
    shortH = (NPTH_CLR + W / 2) - hole_d
    check("hole fixture actually grazes (~17um)", 0.010 < shortH < 0.020)
    compsH = _comps(pcbH, 1)
    n_chH, n_netsH, removedH, addedH = nudge_grazing_microshift(
        [], pcbH, {1}, clearance=CLR3, max_shift=0.025)
    check("hole: pass fired", n_chH > 0 and n_netsH == 1)
    worstH = min(_seg_foreign_hole_dist(pcbH, 1, s.start_x, s.start_y, s.end_x, s.end_y)
                 - (NPTH_CLR + s.width / 2 - 1e-4)
                 for s in pcbH.segments if s.net_id == 1)
    check("hole: graze cleared", worstH >= 0)
    check("hole: connectivity preserved", _comps(pcbH, 1) == compsH)

    # --- HOLE CAP: a hole graze beyond max_shift is left alone ----------------
    npth2 = _pad('J4', 100.0, 100.0, 0.5, 0.5, 2, '/OTHER')
    npth2.drill = 0.5
    npth2.pad_type = 'np_thru_hole'
    gy2 = 100.0 - (HR + NPTH_CLR + W / 2 - 0.035)  # 35um graze -> needs > 25um cap
    ownc = _pad('U3', 98.5, gy2, 0.4, 0.4, 1, '/SIG')
    ownd = _pad('U4', 101.5, gy2, 0.4, 0.4, 1, '/SIG')
    segsHc = [Segment(start_x=98.5, start_y=gy2, end_x=101.5, end_y=gy2,
                      width=W, layer='F.Cu', net_id=1)]
    pcbHc = _board({1: [ownc, ownd], 2: [npth2]}, segsHc, [])
    n_chHc, _, _, addedHc = nudge_grazing_microshift(
        [], pcbHc, {1}, clearance=CLR3, max_shift=0.025)
    check("hole cap: over-cap hole graze left alone", n_chHc == 0 and not addedHc)

    # --- CAP: a graze needing more than max_shift is left alone --------------
    pcb3, foreign3 = _vertex_fixture(pad_x=115.51)  # ~40um past the fixable range
    g3 = [s for s in pcb3.segments if s.width == 0.0889][0]
    short3 = (CLR + g3.width / 2) - _seg_pad_dist(g3, foreign3)
    check("cap fixture needs more than max_shift", short3 + 0.004 > 0.025)
    n_ch3, _, _, added3 = nudge_grazing_microshift(
        [], pcb3, {1}, clearance=CLR, max_shift=0.025)
    check("cap: pass leaves the un-fixable graze alone", n_ch3 == 0 and not added3)

    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s): {fails}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(run())
