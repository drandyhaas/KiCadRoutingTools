#!/usr/bin/env python3
"""The joint escape: the under-pad engine laying a whole-array plan (joint=True).

  python3 tests/test_622_joint_escape_engine.py

generate_bga_fanout(escape_method='jointescape') is how awx's joint fanout
(route_bus --joint-fanout, awx/joint_escape.py) lays its plan: the under-pad
engine with `joint` on, the plan's plane-ball drops laid before anything else
(part 0), and the bus's nets held to their layers (`bus` -> net_layers). This
pins both on kicad_files/ulx3s.kicad_pcb U1, with one signal net fanned
(AUDIO_V2, ball F5) and GND a plane net dropped by the drop pass -- about a
second a run.

1. BASELINE, the liveness check: with no plan GND K10's drop via goes to the
   drop pass's own gap site, which is not the one the plan asks below, and
   AUDIO_V2 escapes on F.Cu. If either stops being so, the arms below test
   nothing, and the test says so rather than passing.
2. PART 0: a plan asking K10's drop in another diagonal gap gets it there,
   the via exactly at the site and its stub on F.Cu from the ball.
   FINE: the plan's own via for that drop (a finer rung of the fab ladder's,
   where the plan found the call's too big for the site) is laid at its size.
   SHARED: a plan dropping K10 and its GND neighbour L10 at one gap site
   (the site as L10 computes it, a hair off K10's) lays ONE via there and
   each ball's stub to it; the old part 0 laid K10's and refused L10's
   (hole to hole), which the drop pass then dropped at a site of its own.
   EDGE and STRAP, on haasoscope_pro_max_test.kicad_pcb IC1 (ulx3s U1's
   corner pads widen its grid past its balls, so none of them is on its
   edge): GND A3's drop planned half a pitch OFF the array is laid there
   (the old part 0 refused any site off the inter-ball gaps), and A3
   strapped to GND B3, which drops, takes a track to B3 and no via.
3. CONTROL: the same plan through the dog-bone engine (the same engine,
   `joint` off) is not read as a drop -- K10's via stays at the drop pass's
   site. Only the joint escape takes a plan's drops.
4. NET LAYERS: AUDIO_V2 held to In1.Cu escapes on In1.Cu alone.

Uses kicad_files/ulx3s.kicad_pcb; skips cleanly if absent.
"""
import contextlib
import io
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from bga_fanout import generate_bga_fanout  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')
LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
SIGNAL, PLANE, NEXT = 'F5', 'K10', 'L10'
# an array whose GND ball EDGE stands on the engine's own grid edge, IN_ (GND) one row in (ulx3s U1's corner pads
# widen its grid a row past its balls, so no ball of it is on the edge)
EDGE_BOARD = os.path.join(ROOT, 'kicad_files', 'haasoscope_pro_max_test.kicad_pcb')
EDGE_REF, EDGE_SIG, EDGE, IN_ = 'IC1', 'A9', 'A3', 'B3'


def run(method='jointescape', hints=None, hold=None, board=None):
    """(the signal ball, the plane ball, tracks, vias, failed) of one fanout of U1: SIGNAL's net fanned, the rest
    either a plane net (dropped) or left alone. `board` 'edge': EDGE_BOARD's EDGE_REF instead (its signal EDGE_SIG,
    its plane ball EDGE), on all its copper layers"""
    edge = board == 'edge'
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(EDGE_BOARD if edge else BOARD)
        fp = pcb.footprints[EDGE_REF if edge else 'U1']
        sig = next(p for p in fp.pads if p.pad_number == (EDGE_SIG if edge else SIGNAL))
        pln = next(p for p in fp.pads if p.pad_number == (EDGE if edge else PLANE))
        bus = {'net_layers': {sig.net_name: hold}, 'priority': [sig.net_name]} if hold else None
        tracks, vias, _vr, failed = generate_bga_fanout(
            fp, pcb, layers=list(pcb.board_info.copper_layers) if edge else LAYERS, track_width=0.12,
            clearance=0.1, via_size=0.35, via_drill=0.2, net_filter=[sig.net_name], escape_method=method,
            plane_drop='auto', escape_dir_hints=hints, bus=bus)
    return sig, pln, tracks, vias, failed


def drop_site(p, vias):
    """p's nearest same-net via within a pitch, or None"""
    vs = [(v['x'], v['y']) for v in vias
          if v['net_id'] == p.net_id and math.hypot(v['x'] - p.global_x, v['y'] - p.global_y) < 0.7]
    return min(vs, key=lambda s: math.hypot(s[0] - p.global_x, s[1] - p.global_y), default=None)


def at(a, b):
    return a is not None and b is not None and math.hypot(a[0] - b[0], a[1] - b[1]) < 1e-6


def ways_down(p, tracks, vias):
    """how many pieces of p's net's copper start at p: tracks with an end on the ball, and a via in its pad"""
    c = (p.global_x, p.global_y)
    return sum(1 for t in tracks if t['net_id'] == p.net_id and (at(t['start'], c) or at(t['end'], c))) + \
        sum(1 for v in vias if v['net_id'] == p.net_id and at((v['x'], v['y']), c))


def layers_of(p, tracks):
    return sorted({t['layer'] for t in tracks if t['net_id'] == p.net_id})


def main():
    print('=' * 60)
    print('joint escape: the plan\'s drops and the bus\'s layers')
    print('=' * 60)
    if not os.path.exists(BOARD):
        print(f'  [SKIP] board not present: {BOARD}')
        return 0
    checks = []
    sig, pln, t0, v0, f0 = run()
    s0 = drop_site(pln, v0)
    # the plan's site: the diagonal gap up and to the left of the ball (the drop pass takes another)
    site = (round(pln.global_x - 0.4, 6), round(pln.global_y + 0.4, 6))
    key = (round(pln.global_x, 3), round(pln.global_y, 3))
    plan = {key: {'kind': 'drop', 'site': site, 'layer': 'F.Cu', 'inpad': False, 'strict': True}}
    print(f'  {PLANE} ({pln.net_name}): drop pass site {s0}, the plan asks {site}; '
          f'{SIGNAL} ({sig.net_name}) on {layers_of(sig, t0)}')
    live = s0 is not None and not at(s0, site) and layers_of(sig, t0) == ['F.Cu'] and not f0
    checks.append(('baseline: the drop pass takes another site, the signal escapes on F.Cu', live))
    if not live:
        print('  [FAIL] baseline moved -- the arms below would test nothing')
        return 1

    _s, _p, t1, v1, _f = run(hints=plan)
    got = drop_site(pln, v1)
    stub = any(t['net_id'] == pln.net_id and t['layer'] == 'F.Cu'
               and {tuple(map(lambda c: round(c, 6), t['start'])), tuple(map(lambda c: round(c, 6), t['end']))}
               == {(round(pln.global_x, 6), round(pln.global_y, 6)), site} for t in t1)
    checks.append(('part 0: the planned drop is laid at its site', at(got, site)))
    checks.append(('part 0: its stub runs on F.Cu from the ball to the site', stub))

    # FINE: the plan's own via for the drop, where it found the call's too big for the site (a finer rung of the fab
    # ladder's: joint_escape._drops) -- laid at that size; the same drop without one, at the call's
    _s, _p, _t7, v7, _f = run(hints={key: dict(plan[key], via=(0.25, 0.15))})
    sized = lambda vs: [(v['size'], v['drill']) for v in vs
                        if v['net_id'] == pln.net_id and math.hypot(v['x'] - site[0], v['y'] - site[1]) < 1e-6]
    checks.append(('fine: a planned drop with the plan\'s own via is laid at that size, without one at the call\'s',
                   sized(v7) == [(0.25, 0.15)] and sized(v1) == [(0.35, 0.2)]))
    print(f'    fine: the drop with the plan\'s via laid {sized(v7)}, without {sized(v1)}')

    # SHARED: the plan drops K10 and its GND neighbour NEXT at ONE gap site (the joint escape's shared drop: plane
    # balls round one barrel, as a human lays ground balls) -- the site as NEXT computes it, a hair off K10's (an
    # array's own pitch error: zynq U5's 0.8001 mm balls put one gap 0.1 um apart from two of them). One via there,
    # each ball's stub to it, and no via of NEXT's own; laid as two drops, the second via is refused (hole to hole)
    # and NEXT goes to the drop pass's own site
    with contextlib.redirect_stdout(io.StringIO()):
        nxt = next(p for p in parse_kicad_pcb(BOARD).footprints['U1'].pads if p.pad_number == NEXT)
    site_n = (nxt.global_x - 0.4 + 2e-7, nxt.global_y - 0.4)
    plan2 = dict(plan)
    plan2[(round(nxt.global_x, 3), round(nxt.global_y, 3))] = {'kind': 'drop', 'site': site_n, 'layer': 'F.Cu',
                                                               'inpad': False, 'strict': True}
    _s, _p, t4, v4, _f = run(hints=plan2)
    there = [v for v in v4 if v['net_id'] == pln.net_id and math.hypot(v['x'] - site[0], v['y'] - site[1]) < 1e-3]
    own_n = ways_down(nxt, t4, v4) - 1        # (its copper: the stub to the shared via and nothing else)
    stubs = [q for q in (pln, nxt) if any(
        t['net_id'] == q.net_id and t['layer'] == 'F.Cu' and at(t['start'], (q.global_x, q.global_y))
        and len(there) == 1 and at(t['end'], (there[0]['x'], there[0]['y'])) for t in t4)]
    print(f'    shared: {len(there)} via(s) at the site, {NEXT} with {own_n} other way(s) down, stubs from '
          f'{[q.pad_number for q in stubs]}')
    checks.append(('shared: two plane balls planned at one gap site take ONE via there, each ball\'s stub to it its '
                   'one way down', len(there) == 1 and len(stubs) == 2 and not own_n))

    # EDGE and STRAP, on EDGE_BOARD's edge ball EDGE (GND) and its GND neighbour IN_ one row in: EDGE's drop planned
    # at its gap half a pitch OFF the array, on the engine grid's edge line (no inter-ball gap: the old part 0 refused
    # it, and the drop pass dropped EDGE at a site of its own); then EDGE strapped to IN_, which drops in a gap of its
    # own -- a track ball to ball, no via of EDGE's
    with contextlib.redirect_stdout(io.StringIO()):
        bx = {p.pad_number: p for p in parse_kicad_pcb(EDGE_BOARD).footprints[EDGE_REF].pads}
    e_, i_ = bx[EDGE], bx[IN_]
    h = math.hypot(e_.global_x - i_.global_x, e_.global_y - i_.global_y) / 2.0       # half the pitch
    ux, uy = (e_.global_x - i_.global_x) / (2 * h), (e_.global_y - i_.global_y) / (2 * h)  # off the array
    site_e = (e_.global_x + h * (ux or 1.0), e_.global_y + h * (uy or 1.0))
    key_e = (round(e_.global_x, 3), round(e_.global_y, 3))
    _s, _p, t5, v5, _f = run(board='edge', hints={key_e: {'kind': 'drop', 'site': site_e, 'layer': 'F.Cu',
                                                          'inpad': False, 'strict': True}})
    got_e = [v for v in v5 if v['net_id'] == e_.net_id and math.hypot(v['x'] - site_e[0], v['y'] - site_e[1]) < 1e-6]
    checks.append(('edge: a planned drop half a pitch off the array is laid there, its stub the ball\'s one way down',
                   len(got_e) == 1 and ways_down(e_, t5, v5) == 1))
    site_i = (i_.global_x - h * (uy or 1.0), i_.global_y - h * (ux or 1.0))       # a gap of IN_'s, further in
    _s, _p, t6, v6, _f = run(board='edge', hints={
        (round(i_.global_x, 3), round(i_.global_y, 3)): {'kind': 'drop', 'site': site_i, 'layer': 'F.Cu',
                                                         'inpad': False, 'strict': True},
        key_e: {'kind': 'strap', 'to': (i_.global_x, i_.global_y), 'layer': 'F.Cu', 'strict': True}})
    strap = any(t['net_id'] == e_.net_id and t['layer'] == 'F.Cu' and {
        tuple(round(c, 6) for c in t['start']), tuple(round(c, 6) for c in t['end'])} == {
        (round(e_.global_x, 6), round(e_.global_y, 6)), (round(i_.global_x, 6), round(i_.global_y, 6))} for t in t6)
    own_e = ways_down(e_, t6, v6) - 1         # (its copper: the strap and nothing else)
    checks.append(('strap: a plane ball strapped to its dropped neighbour takes a track to it, no via',
                   strap and not own_e))
    print(f'    edge ({EDGE_REF} {EDGE}): {len(got_e)} via(s) at its off-array site; strap {EDGE}->{IN_} laid {strap}, '
          f'{EDGE} with {own_e} other way(s) down')

    _s, _p, _t2, v2, _f = run(method='dogbone', hints=plan)
    checks.append(('control: the dog-bone engine does not read the plan\'s drop', at(drop_site(pln, v2), s0)))

    _s, _p, t3, _v3, f3 = run(hold=['In1.Cu'])
    checks.append(('net layers: held to In1.Cu, it escapes on In1.Cu alone',
                   layers_of(sig, t3) == ['In1.Cu'] and not f3))
    print(f'    planned drop laid at {got}; dog-bone laid it at {drop_site(pln, v2)}; '
          f'held, {SIGNAL} on {layers_of(sig, t3)} (failed {f3})')

    fails = 0
    for name, ok in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
        fails += not ok
    print(f'\n{len(checks) - fails}/{len(checks)} checks passed')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
