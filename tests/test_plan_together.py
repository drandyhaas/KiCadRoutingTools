#!/usr/bin/env python3
"""A run's nets planned together (py_router/plan_together.py, KICAD_PLAN_TOGETHER=1).

  python3 tests/test_plan_together.py

The board: two walls of foreign pads cross it, with ONE corridor between them, on the one layer routed -- wide enough
for two tracks side by side at the track pitch and no wider. Two nets, each with a pad above the walls and one below,
must both pass it. A route down the corridor's middle leaves neither side room for a second; two routes pass only
with each to one side. A third net, of three pads, lies clear of the walls.

1. a net planned ALONE runs down the corridor's middle (the change detector for 2: the middle IS where one route
   goes, so two routes a pitch apart have each given way);
2. the two nets planned together both pass the corridor, a pitch apart;
3. the three-pad net's route reaches every pad: two legs, the second from the first;
4. the crate's search (grid_router.plan_search) and the module's own return the same routes;
5. route.py with KICAD_PLAN_TOGETHER=1 routes the three nets by the plan, every net joined, and check_drc finds
   nothing;
6. ...and with no knob there is no plan;
7. the hooks of a step that runs the plan for its own ends: KICAD_PLAN_ONLY=<file> has the plan's report written
   there and the run ended before it routes anything; KICAD_PLAN_ROUNDS and KICAD_PLAN_MAX_PADS are the plan's rounds
   and the most pads a planned net may have.
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile
import uuid

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
from run_utils import tool                                               # noqa: E402

BAD = []
N = [0]
TRACK = CLEAR = 0.15
STEP = 0.1
MID = 10.0                                                               # the corridor's middle (x, mm)


def check(what, ok, detail=''):
    N[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + (f' -- {detail}' if detail else ''))


def board(path, pads, w=20, h=50):
    """a board: `pads` [(ref, x, y, net name)] (through-hole, 0.8 mm) and the outline (0, 0)-(w, h)"""
    U = lambda: str(uuid.uuid4())  # noqa: E731
    nid = {n: i + 1 for i, n in enumerate(sorted({p[3] for p in pads}))}
    L = ['(kicad_pcb', '\t(version 20241229)', '\t(generator "pcbnew")', '\t(generator_version "9.0")',
         '\t(general (thickness 1.6) (legacy_teardrops no))', '\t(paper "A4")',
         '\t(layers (0 "F.Cu" signal) (2 "B.Cu" signal) (5 "F.SilkS" user) (1 "F.Mask" user) (3 "B.Mask" user)'
         ' (25 "Edge.Cuts" user) (35 "F.Fab" user))',
         '\t(setup (pad_to_mask_clearance 0))', '\t(net 0 "")']
    L += [f'\t(net {i} "{n}")' for n, i in nid.items()]
    for ref, x, y, net in pads:
        L += [f'\t(footprint "synth:P" (layer "F.Cu") (uuid "{U()}") (at {x} {y})',
              f'\t\t(property "Reference" "{ref}" (at 0 -1.2 0) (layer "F.SilkS") (uuid "{U()}") (effects (font (size 0.5 0.5) (thickness 0.08))))',
              f'\t\t(pad "1" thru_hole circle (at 0 0) (size 0.8 0.8) (drill 0.4) (layers "*.Cu" "*.Mask") (net {nid[net]} "{net}") (uuid "{U()}"))',
              '\t)']
    for (x1, y1), (x2, y2) in (((0, 0), (w, 0)), ((w, 0), (w, h)), ((w, h), (0, h)), ((0, h), (0, 0))):
        L.append(f'\t(gr_line (start {x1} {y1}) (end {x2} {y2}) (stroke (width 0.1) (type default)) (layer "Edge.Cuts") (uuid "{U()}"))')
    L.append(')')
    with open(path, 'w') as f:
        f.write('\n'.join(L) + '\n')


def pads():
    """the corridor (its walls 1.0 mm apart, copper to copper: track centres fit from 0.275 mm left of the middle to
    0.275 right -- two a pitch apart, none beside one in the middle), the walls across the board, the three nets"""
    out, k = [], 0
    ys = [round(20.0 + 0.9 * i, 2) for i in range(12)]
    for y in ys:                                                         # the corridor's two sides
        for x in (MID - 0.9, MID + 0.9):
            out.append((f'W{k}', round(x, 2), y, 'WALL'))
            k += 1
    row = ys[5]
    xs = [round(MID - 0.9 - 0.9 * i, 2) for i in range(1, 10)] + [0.5] \
        + [round(MID + 0.9 + 0.9 * i, 2) for i in range(1, 10)] + [19.5]
    for x in xs:                                                         # ...and the walls out to the board's edges
        out.append((f'W{k}', x, row, 'WALL'))
        k += 1
    out += [('A1', MID, 10, 'A'), ('A2', MID, 40, 'A'), ('B1', 6, 10, 'B'), ('B2', 6, 40, 'B'),
            ('C1', 4, 4, 'C'), ('C2', 16, 4, 'C'), ('C3', 10, 7, 'C')]
    return out


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)


def in_corridor(legs):
    """the x (mm) of a route's cells halfway down the corridor"""
    return sorted({round(gx * STEP, 3) for way, *_e in legs for gx, gy, _li in way if abs(gy * STEP - 25.0) < 1e-6})


with tempfile.TemporaryDirectory() as td:
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig
    from obstacle_map import build_base_obstacle_map
    import plan_together as pt
    b_in = os.path.join(td, 'in.kicad_pcb')
    board(b_in, pads())
    pcb = quiet(parse_kicad_pcb, b_in)
    ids = {n.name: i for i, n in pcb.nets.items()}
    cfg = GridRouteConfig(layers=['F.Cu'], clearance=CLEAR, track_width=TRACK, via_size=0.5, via_drill=0.3,
                          grid_step=STEP)

    def plan(names, **kw):
        nets = [ids[n] for n in names]
        return pt.plan_routes(pcb, cfg, nets, quiet(build_base_obstacle_map, pcb, cfg, nets), **kw)

    alone, _r = plan(['A'])
    xa = in_corridor(alone.get(ids['A'], []))
    check('1. a net planned alone runs down the corridor\'s middle', xa == [MID], f'at x = {xa}')
    both, rep = plan(['A', 'B', 'C'])
    xa, xb = in_corridor(both.get(ids['A'], [])), in_corridor(both.get(ids['B'], []))
    check('2. the two nets planned together both pass the corridor, a pitch apart',
          len(xa) == 1 and len(xb) == 1 and abs(xa[0] - xb[0]) >= TRACK + CLEAR - 1e-6,
          f'at x = {xa} and {xb}, {rep["rounds"]} round(s)')
    legs = both.get(ids['C'], [])
    ends = {(round(way[-1][0] * STEP, 2), round(way[-1][1] * STEP, 2)) for way, *_e in legs}
    c_pads = {(p.global_x, p.global_y) for p in pcb.nets[ids['C']].pads}
    check('3. the three-pad net\'s route reaches every pad, in two legs',
          len(legs) == 2 and len(ends) == 2 and ends <= c_pads, f'{len(legs)} leg(s), ending at {sorted(ends)}')
    import grid_router
    own, _r = plan(['A', 'B', 'C'], search='py')
    check('4. the crate\'s search and the module\'s own return the same routes',
          hasattr(grid_router, 'plan_search') and own == both and len(both) == 3,
          'the crate has no plan_search' if not hasattr(grid_router, 'plan_search') else f'{len(both)} net(s)')

    def route(tag, env):
        out = os.path.join(td, f'{tag}.kicad_pcb')
        e = {k: v for k, v in os.environ.items() if k != 'KICAD_PLAN_TOGETHER'}
        e.update(env)
        r = subprocess.run([sys.executable, tool('route.py'), b_in, out, '--nets', 'A', 'B', 'C', '--layers', 'F.Cu',
                            '--clearance', str(CLEAR), '--track-width', str(TRACK), '--via-size', '0.5',
                            '--via-drill', '0.3', '--grid-step', str(STEP)], env=e, capture_output=True, text=True)
        return out, r.stdout + r.stderr

    out, log = route('planned', {'KICAD_PLAN_TOGETHER': '1'})
    made = os.path.isfile(out)
    joined, drc = [], '(no board)'
    if made:
        import check_connected as cc
        p = quiet(parse_kicad_pcb, out)
        for name in 'ABC':
            i = {n.name: k for k, n in p.nets.items()}[name]
            r = quiet(cc.check_net_connectivity, i, [s for s in p.segments if s.net_id == i],
                      [v for v in p.vias if v.net_id == i], p.nets[i].pads, [], pcb_data=p)
            joined.append(bool(r.get('connected')))
        d = subprocess.run([sys.executable, tool('check_drc.py'), out, '--clearance', str(CLEAR)],
                           capture_output=True, text=True)
        drc = 'clean' if d.returncode == 0 else (d.stdout + d.stderr)[-400:]
    check('5. route.py with the knob routes the three nets by the plan, every net joined, check_drc clean',
          made and 'Plan: 3 of 3 net(s)' in log and log.count('Routed by the plan') == 3 and joined == [True] * 3
          and drc == 'clean',
          f'{log.count("Routed by the plan")} by the plan, joined {joined}, check_drc {drc}'
          if made else log[-600:])
    _out, log = route('plain', {})
    check('6. with no knob there is no plan', 'Plan:' not in log and 'Routed by the plan' not in log)
    rep = os.path.join(td, 'plan.json')
    out, log = route('only', {'KICAD_PLAN_TOGETHER': '1', 'KICAD_PLAN_ONLY': rep, 'KICAD_PLAN_ROUNDS': '1',
                              'KICAD_PLAN_MAX_PADS': '2'})
    try:
        with open(rep, encoding='utf-8') as f:
            d = json.load(f)
    except (OSError, ValueError):
        d = {}
    check('7. the plan alone: its report written, one round of it, the three-pad net left out, nothing routed',
          d.get('nets') == 2 and d.get('rounds') == 1 and d.get('boxed') == [] and not os.path.isfile(out)
          and 'Routed by the plan' not in log, f'report {d}, a board written: {os.path.isfile(out)}')

print(f'{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
