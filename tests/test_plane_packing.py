#!/usr/bin/env python3
"""route_planes --fewest-plane-layers (py_router/plane_packing.py).

  python3 tests/test_plane_packing.py

Pins, with no board and no routing:

1. unserved_balls: a net's share of a layer is its pours less every other net's pour above it (grown by that
   pour's clearance); a ball outside the piece of it holding the most of its net's pads is unserved -- and so is a
   ball, or a pad the main piece covers, reached only through copper narrower than the net's declared width;
2. pack_plane_layers: the ground family's pairs stand as given; the supplies go first fit onto the plan's supply
   layers, the layers BESIDE a ground plane in the stackup taken last -- freed first, for signals; a layer that can
   hold only one supply frees nothing; the record names the poured pairs and the freed layers.
"""
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import plane_packing as pp  # noqa: E402

fails = []


def check(cond, msg):
    if not cond:
        fails.append(msg)
        print('FAIL', msg)


def sq(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def zone(nid, pts, prio=0, clear=0.1):
    return {'net_id': nid, 'polygon_points': pts, 'priority': prio, 'clearance': clear}


# 1. a 10 mm square of net 1 (the background, priority 0) cut in two by a 2 mm strip of net 2 above it
zones = [zone(1, sq(0, 0, 10, 10)), zone(2, sq(4, 0, 6, 10), prio=1)]
pads = {1: [(1, 1), (1, 9), (2, 5), (8, 5)], 2: [(5, 5)]}
balls = {1: [(8, 5), (1, 1)], 2: [(5, 5)]}
bad = pp.unserved_balls(zones, {1, 2}, balls, pads)
check(bad == {1: [(8, 5)]}, f'1: the ball on the far side of another net\'s strip is unserved, the rest served: {bad}')
check(pp.unserved_balls(zones, {1, 2}, balls, pads, {2: 1.5}) == {1: [(8, 5)]},
      '1: the 2 mm strip serves its ball at a declared 1.5 mm')
check(2 in pp.unserved_balls(zones, {1, 2}, balls, pads, {2: 2.5}),
      '1: ...and not at a declared 2.5 mm: the whole strip narrower than the width')
# a pad the pour covers, reached only through a 0.3 mm neck: two squares of net 3 joined by it
neck = [(0, 0), (4, 0), (4, 1.85), (6, 1.85), (6, 0), (10, 0), (10, 4), (6, 4), (6, 2.15), (4, 2.15), (4, 4), (0, 4)]
zones3 = [zone(3, neck)]
pads3 = {3: [(1, 1), (2, 2), (8, 2)]}
check(pp.unserved_balls(zones3, {3}, {3: []}, pads3, {3: 0.2}) == {},
      '1: a pad behind a 0.3 mm neck is served at a declared 0.2 mm')
check(pp.unserved_balls(zones3, {3}, {3: []}, pads3, {3: 0.5}) == {3: [(8, 2)]},
      '1: ...and unserved at a declared 0.5 mm, though it is no ball')

# 2. the packing, on a stand-in board: GND on In1 and In6, four supplies on In2..In5, a layer holding two supplies
stack = ['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'In5.Cu', 'In6.Cu', 'B.Cu']
names = ['GND', 'GND', 'S1', 'S2', 'S3', 'S4']
plan_layers = ['In1.Cu', 'In6.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'In5.Cu']
ids = {'GND': 1, 'S1': 11, 'S2': 12, 'S3': 13, 'S4': 14}
pcb = types.SimpleNamespace(nets={i: types.SimpleNamespace(name=n, net_id=i) for n, i in ids.items()}, pads_by_net={},
                            board_info=types.SimpleNamespace(copper_layers=stack))
pp.ball_positions = lambda _pcb: {11: [(1, 0)], 12: [(2, 0)], 13: [(3, 0)], 14: [(4, 0)]}
pp.pad_positions = lambda _pcb: {11: [(1, 0)], 12: [(2, 0)], 13: [(3, 0)], 14: [(4, 0)]}
seen = []


def stub_split(cap):
    def _split(_f, layer, nets, plane_kw, pcb_data=None):
        # (each net a 1 mm square round its ball; past `cap` nets on a layer, the extra ones get none)
        seen.append((layer, tuple(nets), plane_kw.get('power_nets_widths')))
        out = []
        for k, n in enumerate(nets):
            i = ids[n]
            if k < cap:
                out.append(zone(i, sq(i % 10 - 0.5, -0.5, i % 10 + 0.5, 0.5), prio=k))
        return out
    return _split


pp._split = stub_split(2)
kw = {'power_nets': ['S1', 'S2', 'S3', 'S4'], 'power_nets_widths': [0.2, 0.2, 0.2, 0.2]}
n2, L2, ln, rec = pp.pack_plane_layers('', names, plan_layers, kw, pcb_data=pcb, log=lambda *a: None)
check([(n, L) for n, L in zip(n2, L2) if n == 'GND'] == [('GND', 'In1.Cu'), ('GND', 'In6.Cu')],
      f'2: the ground pairs stand as given: {list(zip(n2, L2))}')
check(set(ln) == {'In1.Cu', 'In6.Cu', 'In3.Cu', 'In4.Cu'} and sorted(ln['In3.Cu'] + ln['In4.Cu']) == ['S1', 'S2', 'S3', 'S4'],
      f'2: four supplies on the two layers away from ground: {ln}')
check(rec.get('freed') == ['In2.Cu', 'In5.Cu'] and rec.get('freed_beside_ground') == ['In2.Cu', 'In5.Cu'],
      f'2: the layers beside a ground plane freed: {rec}')
check(sorted(map(tuple, rec.get('pairs', []))) == sorted(zip(n2, L2)), '2: the record names the poured pairs')
check(all(w == [0.2, 0.2, 0.2, 0.2] for _L, _n, w in seen), '2: the trial splits run on the caller\'s own settings')
pp._split = stub_split(1)
n3, L3, ln3, rec3 = pp.pack_plane_layers('', names, plan_layers, kw, pcb_data=pcb, log=lambda *a: None)
check(rec3.get('freed') == [] and len(set(L3)) == 6, f'2: one supply to a layer frees none: {rec3}')

print('plane_packing: ' + ('FAIL' if fails else 'PASS: a pour serves a ball through copper no narrower than its '
                           'width; the supplies packed, the layers beside ground freed first'))
sys.exit(1 if fails else 0)
