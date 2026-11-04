#!/usr/bin/env python3
"""Every escape the channel BGA fanout keeps reaches its ball.

  python3 tests/test_bga_fanout_escape_reaches_ball.py

An escape off the top layer reaches its SMD ball only through a via in the
ball. Three ways the channel engine shipped one without it, the ball still
counted as escaped:

1. The pad-aware repair's alternate-channel and jogged reroutes replace the
   route with a NEW one (a new track uid) and rebuilt only the new route's
   copper: the old route's tracks stayed, on their old layer, while
   manage_vias placed vias from the new route's layer.
2. rebalance_layers moved a route's tracks by flood-filling the net's copper
   from its pad, which runs on into a same-net escape that touches it (a
   neighbour link ends on its anchor's ball) and moved that copper while its
   route kept the old layer.
3. A neighbour link runs ball to ball; off the top layer it needs a via in the
   ball it ends on too, which only the anchor ball's own route placed, while
   that route stayed off the top layer as well. The layer passes move the two
   independently.

The board checks walk the copper a real fanout returns: every connected group
of its tracks and vias must touch a ball (a via in it, or a track end on the
ball's own layer), and a track that ends at a ball on another layer needs a
via there. The board's own vias count as the ball's. The unit check drives
rebalance_layers on a link and its anchor.
"""
import contextlib
import io
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

KF = os.path.join(ROOT, 'kicad_files')
# (board, part): each fanned out by the channel engine, alone, at the BGA
# defaults, on every copper layer of the board.
CASES = [
    ('glasgow_revC.kicad_pcb', 'U30'),            # a repair reroute (1)
    ('haasoscope_pro_max_test.kicad_pcb', 'U1'),  # links under top anchors (3)
    ('haasoscope_pro_max_test.kicad_pcb', 'IC1'),
]
TOL = 1e-3


def _on_seg(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(p[0] - a[0], p[1] - a[1]) < TOL
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(a[0] + t * dx - p[0], a[1] + t * dy - p[1]) < TOL


def _in_pad(p, pad):
    return (abs(p[0] - pad.global_x) <= pad.size_x / 2 + TOL
            and abs(p[1] - pad.global_y) <= pad.size_y / 2 + TOL)


def walk(footprint, tracks, vias, board_vias):
    """(groups of returned copper touching no ball, balls reached on a layer
    they are not on with no via in them)."""
    items = [('t', t) for t in tracks] + [('v', v) for v in vias]
    par = list(range(len(items)))

    def find(i):
        while par[i] != i:
            par[i] = par[par[i]]
            i = par[i]
        return i
    by_net = {}
    for i, (_k, o) in enumerate(items):
        by_net.setdefault(o['net_id'], []).append(i)
    for idx in by_net.values():
        for x in range(len(idx)):
            ka, a = items[idx[x]]
            for y in range(x + 1, len(idx)):
                kb, b = items[idx[y]]
                if ka == 't' and kb == 't':
                    hit = a['layer'] == b['layer'] and (
                        any(_on_seg(p, b['start'], b['end']) for p in (a['start'], a['end']))
                        or any(_on_seg(p, a['start'], a['end']) for p in (b['start'], b['end'])))
                elif ka == 'v' and kb == 'v':
                    hit = math.hypot(a['x'] - b['x'], a['y'] - b['y']) < TOL
                else:
                    t, v = (a, b) if ka == 't' else (b, a)
                    hit = any(math.hypot(p[0] - v['x'], p[1] - v['y']) <= v['size'] / 2 + TOL
                              for p in (t['start'], t['end']))
                if hit:
                    par[find(idx[x])] = find(idx[y])
    anchored, dangling = set(), []
    for pad in footprint.pads:
        if not pad.net_id:
            continue
        cu = {l for l in (pad.layers or []) if l.endswith('.Cu')}
        every = (pad.drill or 0) > 0 or any(l.startswith('*') for l in cu)
        mine = by_net.get(pad.net_id, [])
        via_in = [i for i in mine if items[i][0] == 'v'
                  and _in_pad((items[i][1]['x'], items[i][1]['y']), pad)]
        board_via = any(v.net_id == pad.net_id and _in_pad((v.x, v.y), pad) for v in board_vias)
        for i in via_in:
            anchored.add(find(i))
        for i in mine:
            k, o = items[i]
            if k != 't' or not any(_in_pad(p, pad) for p in (o['start'], o['end'])):
                continue
            if every or o['layer'] in cu or board_via:
                anchored.add(find(i))
            elif not via_in:
                dangling.append(f"{pad.pad_number} ({pad.net_name}) on {o['layer']}")
    groups = {}
    for i in range(len(items)):
        if find(i) not in anchored:
            groups.setdefault(find(i), []).append(items[i])
    return list(groups.values()), sorted(set(dangling))


def check_boards(fails):
    from kicad_parser import parse_kicad_pcb
    import bga_fanout
    import routing_defaults as d
    for board, ref in CASES:
        path = os.path.join(KF, board)
        if not os.path.isfile(path):
            fails.append(f"missing board {path}")
            continue
        pcb = parse_kicad_pcb(path)
        fp = pcb.footprints[ref]
        with contextlib.redirect_stdout(io.StringIO()):
            tracks, vias, _rm, _failed = bga_fanout.generate_bga_fanout(
                fp, pcb, layers=pcb.board_info.copper_layers, escape_method='channel',
                track_width=d.BGA_TRACK_WIDTH, clearance=d.BGA_CLEARANCE,
                via_size=d.BGA_VIA_SIZE, via_drill=d.BGA_VIA_DRILL)
        if not tracks or not any(t['layer'] != pcb.board_info.copper_layers[0] for t in tracks):
            fails.append(f"{board} {ref}: no escape off the top layer -- the fixture tests nothing")
            continue
        groups, dangling = walk(fp, tracks, vias, pcb.vias)
        for g in groups:
            k, o = g[0]
            at = o['start'] if k == 't' else (o['x'], o['y'])
            fails.append(f"{board} {ref}: {len(g)} piece(s) of net {o['net_id']} copper at "
                         f"({at[0]:.2f}, {at[1]:.2f}) reach no ball")
        if dangling:
            fails.append(f"{board} {ref}: escape copper ends at {len(dangling)} ball(s) with no "
                         f"via in them: {dangling}")
        print(f"  {board} {ref}: {len(tracks)} tracks, {len(vias)} vias, every piece reaches a ball"
              if not groups and not dangling else f"  {board} {ref}: FAIL")


def check_rebalance(fails):
    """A link and its anchor on one layer; rebalancing moves the anchor. The
    link's track must stay on the link route's layer."""
    from kicad_parser import Pad
    from bga_fanout.types import FanoutRoute, route_uid
    from bga_fanout.tracks import generate_tracks_from_routes
    from bga_fanout.layer_balance import rebalance_layers

    def pad(net, x, y, num):
        return Pad(component_ref='U1', pad_number=num, global_x=x, global_y=y, local_x=x,
                   local_y=y, size_x=0.35, size_y=0.35, shape='circle', layers=['F.Cu'],
                   net_id=net, net_name=f"N{net}")
    pa, pl, pc = pad(1, 0.0, 0.0, 'A1'), pad(1, 0.8, 0.0, 'A2'), pad(2, 0.0, 8.0, 'J1')
    anchor = FanoutRoute(pad=pa, pad_pos=(0.0, 0.0), stub_end=(-2.0, 0.0), exit_pos=(-2.0, 0.0),
                         is_edge=True, escape_dir='left', layer='B.Cu')
    link = FanoutRoute(pad=pl, pad_pos=(0.8, 0.0), stub_end=(0.0, 0.0), exit_pos=(0.0, 0.0),
                       is_edge=True, layer='B.Cu', neighbor_connection=True)
    other = FanoutRoute(pad=pc, pad_pos=(0.0, 8.0), stub_end=(-2.0, 8.0), exit_pos=(-2.0, 8.0),
                        is_edge=True, escape_dir='left', layer='B.Cu')
    routes = [anchor, link, other]
    with contextlib.redirect_stdout(io.StringIO()):
        tracks, _e, _i = generate_tracks_from_routes(routes, 0.1, 'F.Cu')
        moved = rebalance_layers(routes, tracks, [], ['F.Cu', 'B.Cu'], 0.2)
    if not moved or anchor.layer != 'F.Cu' or link.layer != 'B.Cu':
        fails.append(f"rebalance: expected the anchor alone moved to F.Cu (moved {moved}, anchor "
                     f"{anchor.layer}, link {link.layer}) -- the fixture tests nothing")
        return
    by_uid = {route_uid(r): r for r in routes}
    off = [(by_uid[t['route_uid']].pad.pad_number, t['layer'], by_uid[t['route_uid']].layer)
           for t in tracks if t['layer'] != by_uid[t['route_uid']].layer]
    if off:
        fails.append(f"rebalance: track(s) left on a layer their route is not on "
                     f"(ball, track layer, route layer): {off}")
    else:
        print("  rebalance: the anchor moved alone; every track stayed on its route's layer")


def main():
    print('=' * 60)
    print("BGA channel escape: every escape kept reaches its ball")
    print('=' * 60)
    fails = []
    check_rebalance(fails)
    check_boards(fails)
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print("PASS: no escape copper is cut off from its ball")
    return 0


if __name__ == '__main__':
    sys.exit(main())
