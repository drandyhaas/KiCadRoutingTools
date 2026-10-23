#!/usr/bin/env python3
"""A BGA escape the channel engine gives up takes ONE ball's copper, not its net's.

  python3 tests/test_bga_fanout_drop_one_ball.py

Three places in `_generate_bga_fanout_core` drop a ball's escape -- a
via-in-pad manage_vias cannot place (#253), an escape that cannot clear a
foreign pad even at its shortest (#123 PAD-SEGMENT), a via that would short a
foreign track (#123 "Via-clearance") -- and all three removed copper by NET:
one ball took every escape of a multi-ball rail with it (#508 finding 6's
class), and the last two left the rail's routes counted as escaped. Each is
driven here on a real channel pass (glasgow_revC U30, one pass, no retries),
made to give up exactly ONE ball of a net that has other balls with vias;
that ball's tracks and via go, every other ball of the net keeps its own.
"""
import io
import contextlib
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

BOARD = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']


def _pick(routes, vias):
    """A route of a net with >= 2 routes carrying a via-in-pad, and its
    siblings: [(uid, pad_pos)] for the target and the rest."""
    from bga_fanout.types import route_uid
    at = {(round(v['x'], 6), round(v['y'], 6), v['net_id']) for v in vias}
    viad = [r for r in routes if r.pair_id is None
            and (round(r.pad_pos[0], 6), round(r.pad_pos[1], 6), r.net_id) in at]
    by = {}
    for r in viad:
        by.setdefault(r.net_id, []).append(r)
    nid = min(n for n, rs in by.items() if len(rs) >= 2)
    rs = sorted(by[nid], key=lambda r: r.pad_pos)
    return rs[0], [(route_uid(r), r.pad_pos) for r in rs[1:]]


def _run(pcb, fp):
    import bga_fanout
    with contextlib.redirect_stdout(io.StringIO()):
        return bga_fanout._generate_bga_fanout_core(
            fp, pcb, layers=LAYERS, escape_method='channel', _single_pass=True)


def _judge(name, state, result, fails):
    from bga_fanout.types import route_uid
    if 'target' not in state:
        fails.append(f"{name}: the drop was never driven (fixture)")
        return
    tracks, vias = result[0], result[1]
    tgt = state['target']
    uid = route_uid(tgt)
    if any(t.get('route_uid') == uid for t in tracks) or any(
            abs(v['x'] - tgt.pad_pos[0]) < 1e-6 and abs(v['y'] - tgt.pad_pos[1]) < 1e-6
            and v['net_id'] == tgt.net_id for v in vias):
        fails.append(f"{name}: the given-up ball {tgt.pad.pad_number} still has its copper")
    lost = [pp for u, pp in state['siblings']
            if not any(t.get('route_uid') == u for t in tracks)
            or not any(abs(v['x'] - pp[0]) < 1e-6 and abs(v['y'] - pp[1]) < 1e-6
                       for v in vias)]
    if lost:
        fails.append(f"{name}: giving up ball {tgt.pad.pad_number} of {tgt.pad.net_name} also "
                     f"removed the escape of {len(lost)} of its {len(state['siblings'])} "
                     f"sibling ball(s)")


def check_via_clearance(fails):
    """Report one via as shorting, at the via-clearance check itself."""
    from kicad_parser import parse_kicad_pcb
    import geometry_utils
    orig = geometry_utils.point_to_segment_distance
    state = {}

    def spy(px, py, *a):
        loc = sys._getframe(1).f_locals
        if 'via_clear_tol' in loc and 'best_routes' in loc and 'via' in loc:
            if 'target' not in state:
                state['target'], state['siblings'] = _pick(loc['best_routes'],
                                                           loc['vias_to_add'])
            t = state['target']
            v = loc['via']
            if v['net_id'] == t.net_id and abs(v['x'] - t.pad_pos[0]) < 1e-6 \
                    and abs(v['y'] - t.pad_pos[1]) < 1e-6:
                return 0.0
        return orig(px, py, *a)
    geometry_utils.point_to_segment_distance = spy
    try:
        pcb = parse_kicad_pcb(BOARD)
        res = _run(pcb, pcb.footprints['U30'])
    finally:
        geometry_utils.point_to_segment_distance = orig
    _judge('via-clearance', state, res, fails)


def check_escape_clearing(fails):
    """The escape-clearing step gives up one route."""
    from kicad_parser import parse_kicad_pcb
    import bga_fanout
    orig = bga_fanout.clear_escapes_of_foreign_pads
    state = {}

    def fake(routes, tracks, *a, **k):
        n, dropped = orig(routes, tracks, *a, **k)
        loc = sys._getframe(1).f_locals
        state['target'], state['siblings'] = _pick(routes, loc['vias_to_add'])
        if k.get('dropped_routes') is not None:
            k['dropped_routes'].append(state['target'])
        return n, list(dropped) + [state['target'].pad.net_name]
    bga_fanout.clear_escapes_of_foreign_pads = fake
    try:
        pcb = parse_kicad_pcb(BOARD)
        res = _run(pcb, pcb.footprints['U30'])
    finally:
        bga_fanout.clear_escapes_of_foreign_pads = orig
    _judge('escape-clearing', state, res, fails)


def check_via_blocked(fails):
    """manage_vias refuses one route's via-in-pad (#253)."""
    from kicad_parser import parse_kicad_pcb
    import bga_fanout
    orig = bga_fanout.manage_vias
    state = {}

    def fake(routes, *a, **k):
        add, rem, blocked = orig(routes, *a, **k)
        if 'target' not in state:
            state['target'], state['siblings'] = _pick(routes, add)
        t = state['target']
        if t in routes:
            add = [v for v in add if not (v['net_id'] == t.net_id
                                          and abs(v['x'] - t.pad_pos[0]) < 1e-6
                                          and abs(v['y'] - t.pad_pos[1]) < 1e-6)]
            blocked = list(blocked) + [t]
        return add, rem, blocked
    bga_fanout.manage_vias = fake
    try:
        pcb = parse_kicad_pcb(BOARD)
        res = _run(pcb, pcb.footprints['U30'])
    finally:
        bga_fanout.manage_vias = orig
    _judge('via-blocked', state, res, fails)


def main():
    print('=' * 60)
    print("BGA channel escape: a given-up ball takes its own copper, not its net's")
    print('=' * 60)
    fails = []
    if not os.path.isfile(BOARD):
        fails.append(f"missing board {BOARD}")
    else:
        check_via_clearance(fails)
        check_escape_clearing(fails)
        check_via_blocked(fails)
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print("PASS: via-clearance, escape-clearing and via-blocked drops each take the one "
          "ball's tracks and via; its net's other balls keep theirs")
    return 0


if __name__ == '__main__':
    sys.exit(main())
