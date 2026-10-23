#!/usr/bin/env python3
"""The ESCAPE fanout's pieces (escape_plan.py, dogbone_arrays.py; route_bus --escape).

  python3 tests/test_622_escape_plan.py

On a board of its own -- a 5 x 5 ball array at 0.8 mm pitch (0.4 mm round balls on F.Cu), four copper layers, every
ball netted -- at the zynq LVDS run's sizes (clearance 0.09, escape track 0.12, via 0.45/0.3), this pins:

1. dogbone_arrays: every ball dog-boned -- a stub on F.Cu from the ball to a via on one of its four diagonal gap points,
   none in its pad (a 0.45 via fits a 0.8 mm gap at 0.09), every via clear of every other ball and via;
2. escape_plan.escape_menu, from a ball's via: every move a 'dogbone' whose first leg is that stub on F.Cu and whose
   runs are on another layer, axis-aligned (the selector prices them as lanes: `climb` set), ending a pitch past the
   outer balls (PAST) on a BALL line -- the free lines of a dog-boned field -- or, a STREET escape, on a gap line;
3. the moves keep the escape track and clearance off every other net's via and ball on their layer;
4. SIDESTEPS: lines a pitch and more past the two beside the via, reached along the ball row half a pitch out;
5. a foreign via standing on a ball line takes that line's moves out of the menu;
6. select_moves._conflict: two exits stacked at one point on two layers are no conflict (the whole route stacks them),
   two at one point on one layer are;
7. pages_first.current_tooth: no tooth on an ESCAPE state (its dog-boned base lays none);
8. STREETS: U1's rows dog-boned out of its middle leave the gap line between rows 1 and 2 free of vias -- a ball
   beside it escapes along it both ways, its exit on that gap line; a ball whose two neighbouring gap lines carry
   vias has no street escape;
9. the plan sidecar carries the ends model's reading of each lane (its nets over two, load, crossings, fronts) as
   fanout_from_plan's does: a round the whole route lays nothing on names lanes from it (whole_feedback --name), and
   with it empty the zynq LVDS bus's 47 nets on escapes named none -- three rounds laid the same ends;
10. a net with no tooth escape (its via walled in) is no lane of the ends model, as one with no berth is: the run
   leaves it out and names it (the joint ends had crashed on it);
11. dogbone_arrays: a NO-CONNECT ball -- its net no other pad on the board -- is left bare, no via and no stub, as
   bga_fanout fans none (#122), every other ball dog-boned as before (h616_lpddr4's H616 NC balls, dog-boned, put
   vias on the decaps beneath it and the bus step refused its own board). Each other ball's net has a second pad, on
   a connector J1 far off the arrays.
"""
import contextlib
import io
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('AWX_RULES', None)

import rules as _rules  # noqa: E402
_rules.install(_rules.Rules(clearance=0.09, track=0.15, via_size=0.45, via_drill=0.3, fan_track=0.12,
                            hole_to_hole=0.25, source='test'))
import escape_moves as em  # noqa: E402
import escape_plan as ep  # noqa: E402
import dogbone_arrays as dga  # noqa: E402
import select_moves as sm  # noqa: E402
import pages_first as pf  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

N, PITCH = 5, 0.8


def board(extra='', nc=()):
    nets = '\n'.join(f'  (net {i + 1} "N{i}")' for i in range(N * N))
    pads = '\n'.join(f'    (pad "{i}" smd circle (at {(i % N) * PITCH:.3f} {(i // N) * PITCH:.3f}) (size 0.4 0.4) '
                     f'(layers "F.Cu" "F.Mask") (net {i + 1} "N{i}"))' for i in range(N * N))
    # (each ball's net a second pad on J1, 30 mm off -- a ball whose net has none is a no-connect; N0 meets U2 too)
    jpads = '\n'.join(f'    (pad "{i}" smd rect (at {i * 1.0:.3f} 0) (size 0.5 0.5) (layers "F.Cu" "F.Mask") '
                      f'(net {i + 1} "N{i}"))' for i in range(1, N * N) if i not in nc)
    return f"""(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
{nets}
  (net {N * N + 1} "OTHER")
  (footprint "BGA" (layer "F.Cu") (at 0 0)
    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))
{pads}
  )
  (footprint "BGA2" (layer "F.Cu") (at 20 0)
    (property "Reference" "U2" (at 0 0) (layer "F.SilkS"))
    (pad "1" smd circle (at 0 0) (size 0.4 0.4) (layers "F.Cu" "F.Mask") (net 1 "N0"))
  )
  (footprint "CONN" (layer "F.Cu") (at -30 30)
    (property "Reference" "J1" (at 0 0) (layer "F.SilkS"))
{jpads}
  )
{extra}
)
"""


def parse(text):
    d = tempfile.mkdtemp()
    p = os.path.join(d, 'b.kicad_pcb')
    with open(p, 'w') as f:
        f.write(text)
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(p), p


fails = []


def check(cond, msg):
    if not cond:
        fails.append(msg)
        print('FAIL', msg)


# 1. dog-bones
pcb, path = parse(board())
with contextlib.redirect_stdout(io.StringIO()):
    tracks, vias, stats = dga.dogbone(pcb, ['U1', 'U2'], log=lambda *a: None)
u1 = [v for v in vias if v['net_id'] <= N * N and abs(v['x'] - 20) > 1]
check(stats['U1'][0] == N * N and stats['U1'][1] == 0 and not stats['U1'][2],
      f'1: every U1 ball dog-boned, none in its pad: {stats["U1"]}')
for v in u1:
    fx, fy = v['x'] / PITCH, v['y'] / PITCH
    check(abs(fx - round(fx - 0.5) - 0.5) < 1e-6 and abs(fy - round(fy - 0.5) - 0.5) < 1e-6,
          f'1: via of net {v["net_id"]} at ({v["x"]:.3f},{v["y"]:.3f}) on a gap point')
for i, a in enumerate(u1):
    for b in u1[i + 1:]:
        check(math.hypot(a['x'] - b['x'], a['y'] - b['y']) >= 0.45 + 0.09 - 1e-6,
              f'1: vias of nets {a["net_id"]} and {b["net_id"]} {math.hypot(a["x"] - b["x"], a["y"] - b["y"]):.3f} apart')

# 11. a no-connect ball left bare
NCB = N * N // 2                                      # the middle ball (no outer ball's way out), its net on no other pad
pcb_nc, _p = parse(board(nc=(NCB,)))
with contextlib.redirect_stdout(io.StringIO()):
    t_nc, v_nc, st_nc = dga.dogbone(pcb_nc, ['U1', 'U2'], log=lambda *a: None)
check(not any(v['net_id'] == NCB + 1 for v in v_nc) and not any(t['net_id'] == NCB + 1 for t in t_nc),
      '11: the no-connect ball has no dog-bone via or stub')
check(st_nc['U1'][0] == N * N - 1 and not st_nc['U1'][2],
      f'11: every other U1 ball dog-boned: {st_nc["U1"]}')

# the dog-boned board, for the menus
vtxt = '\n'.join(f'  (via (at {v["x"]:.4f} {v["y"]:.4f}) (size 0.45) (drill 0.3) (layers "F.Cu" "B.Cu") '
                 f'(net {v["net_id"]}))' for v in vias)
stxt = '\n'.join(f'  (segment (start {t["start"][0]:.4f} {t["start"][1]:.4f}) (end {t["end"][0]:.4f} {t["end"][1]:.4f}) '
                 f'(width 0.12) (layer "F.Cu") (net {t["net_id"]}))' for t in tracks)
pcb, path = parse(board(vtxt + '\n' + stxt))
r = _rules.active()
copper = ep.Copper(pcb, r.fan_track, r.fan_clear)
grid = em.grid_of(pcb.footprints['U1'])
pad = next(p for p in pcb.footprints['U1'].pads if p.pad_number == str(1 * N + 2))     # row 1, column 2
site = ep.own_via(pcb, pad, pad.net_id, grid)
check(site is not None, '2: the ball has its dog-bone via')
layers = ['F.Cu', 'B.Cu', 'In1.Cu', 'In2.Cu']
moves = ep.escape_menu(pad, site, grid, ['right', 'down', 'up', 'left'], layers, copper)
check(len(moves) > 0, '2: the ball has escapes')
x1 = grid.bbox[2]
for m in moves:
    check(m.kind == 'dogbone' and m.climb == 1, f'2: {m.direction} {m.layer}: a dogbone with climb set')
    check(m.legs[0][2] == 'F.Cu' and m.legs[0][1] == site, '2: the first leg the stub on F.Cu to the via')
    check(m.layer != 'F.Cu' and all(L == m.layer for _p, _q, L in m.legs[1:]), '2: the runs off F.Cu, on its layer')
    for p, q, _L in m.legs[1:]:
        check(abs(p[0] - q[0]) < 1e-9 or abs(p[1] - q[1]) < 1e-9, f'2: run {p}->{q} axis-aligned')
    along = m.exit_pt[1] if m.direction in ('left', 'right') else m.exit_pt[0]
    on_gap = abs(along / PITCH - round(along / PITCH - 0.5) - 0.5) < 1e-6
    check(abs(along / PITCH - round(along / PITCH)) < 1e-6 or (getattr(m, 'street', False) and on_gap),
          f'2: exit {m.exit_pt} on a ball line, or a street escape\'s on a gap line')
    if m.direction == 'right':
        check(abs(m.exit_pt[0] - (x1 + PITCH)) < 1e-6, f'2: a right exit a pitch past the outer balls: {m.exit_pt}')
# 3. clear of other nets' vias and balls on its layer
for m in moves:
    for p, q, L in m.legs[1:]:
        for v in pcb.vias:
            if v.net_id != pad.net_id:
                check(ep._pt_seg((v.x, v.y), p, q) >= v.size / 2 + r.fan_track / 2 + r.fan_clear - 1e-6,
                      f'3: {m.direction} {L} run past via of net {v.net_id}')
# 4. sidesteps
check(any(getattr(m, 'esc_side', 0) >= 1 for m in moves), '4: sidestep moves offered')
# 5. a foreign via on a ball line takes that line out
lines_right = {round(m.exit_pt[1], 3) for m in moves if m.direction == 'right'}
target = min(lines_right, key=lambda y: abs(y - site[1]))
blk = f'  (via (at {x1 + PITCH / 2 + 0.05:.4f} {target:.4f}) (size 0.45) (drill 0.3) (layers "F.Cu" "B.Cu") (net {N * N + 1}))'
pcb2, _ = parse(board(vtxt + '\n' + stxt + '\n' + blk))
moves2 = ep.escape_menu(pad, site, grid, ['right'], layers, ep.Copper(pcb2, r.fan_track, r.fan_clear))
check(not any(abs(m.exit_pt[1] - target) < 1e-6 for m in moves2),
      f'5: no right escape on the blocked line {target}')
check(len(moves2) > 0, '5: the other lines still offered')
# 6. stacked exits and no_stack
a = em.Move('A', 'dogbone', 'right', 'B.Cu', (5.0, 1.6), 1, [((0, 0), (0.4, 0.4), 'F.Cu'), ((0.4, 1.6), (5.0, 1.6), 'B.Cu')],
            site=(0.4, 0.4), climb=1)
b = em.Move('B', 'dogbone', 'right', 'In1.Cu', (5.0, 1.6), 1,
            [((0.8, 0), (1.2, 0.4), 'F.Cu'), ((1.2, 1.6), (5.0, 1.6), 'In1.Cu')], site=(1.2, 0.4), climb=1)
check(not sm._conflict(a, b, strict=True, stack=True), '6: stacked exits on two layers: no conflict')
b.layer = 'B.Cu'
b.legs = [((0.8, 0), (1.2, 0.4), 'F.Cu'), ((1.2, 1.6), (5.0, 1.6), 'B.Cu')]
b.__dict__.pop('_spans_c', None)
check(sm._conflict(a, b, strict=True, stack=True), '6: two exits at one point on ONE layer: a conflict')
# 7. no tooth on an escape state
st = {'src_pad': {'N7': pad}, 'sref': 'U1', 'escape': True, 'pcb': pcb, 'byname': {}}
check(pf.current_tooth(st, 'N7') is None, '7: an escape state lays no tooth')

# 8. streets: the gap line between rows 1 and 2 (y 1.2) free of vias, the test ball (row 1) beside it
gy = 1.5 * PITCH
check(not any(abs(v.y - gy) < 1e-6 for v in pcb.vias if v.net_id <= N * N),
      '8: no U1 via on the gap line between rows 1 and 2 (the field dog-boned out of its middle)')
st_ = [m for m in moves if getattr(m, 'street', False)]
check({m.direction for m in st_} >= {'left', 'right'} and all(abs(m.exit_pt[1] - gy) < 1e-6 for m in st_),
      f'8: the ball beside the street escapes along it both ways: {[(m.direction, m.exit_pt) for m in st_]}')
dpad = next(p for p in pcb.footprints['U1'].pads if p.pad_number == str(3 * N + 2))     # row 3, column 2
dmoves = ep.escape_menu(dpad, ep.own_via(pcb, dpad, dpad.net_id, grid), grid, ['right', 'left'], layers, copper)
check(dmoves and not any(getattr(m, 'street', False) for m in dmoves),
      f'8: a ball with vias on both its gap lines has no street escape: {len(dmoves)} moves')
# 9. the sidecar carries the ends model's reading of the lanes
import json  # noqa: E402
with tempfile.TemporaryDirectory() as td_:
    out_ = os.path.join(td_, 'fo.kicad_pcb')
    b.layer = 'In1.Cu'
    # (as whole_ends.choose.last holds it: the chosen state's lane readings under 'parts')
    read_ = {'parts': {'lane_over': {'A': 1}, 'lane_load': {'A': 0.9, 'B': 0.2}, 'lane_x': {'A': 3, 'B': 1},
                       'lane_front': {'B': {'1': [0.1, 0.2, 0.3, 0.4]}}}, 'cut': ['S', 1.0]}
    ep.sidecar(out_, {'chi': 1}, ['A', 'B'], {'A': a, 'B': b}, {'A': b, 'B': a}, read_['cut'], read_)
    em_ = json.load(open(os.path.join(td_, 'fo.plan.json'))).get('ends_model') or {}
    check(em_ == {'over': {'A': 1}, 'load': {'A': 0.9, 'B': 0.2}, 'x': {'A': 3, 'B': 1},
                  'front': {'B': {'1': [0.1, 0.2, 0.3, 0.4]}}},
          f'9: the sidecar\'s ends_model is the ends model\'s reading: {em_}')

# 10. a lane with no tooth escape is no lane: N0 (U1 to U2) with every move from its via banned -- the ends model
# leaves it out, as it does a lane with no berth (joint_choose had crashed on it: zynq U1's walled-in CTRL_OUT0)
import whole_ends  # noqa: E402
import source_realize as sr  # noqa: E402
with contextlib.redirect_stdout(io.StringIO()):
    st0 = ep.plan_state(pcb, ['N0'], 'U1', 'U2', log=lambda *a: None)
    lanes0 = [l_ for l_, _lg in whole_ends.Ends(st0).lanes]
    st1 = ep.plan_state(pcb, ['N0'], 'U1', 'U2', log=lambda *a: None,
                        banned={('N0', sr.move_sig(m)) for m in st0['smenu']['N0']})
    lanes1 = [l_ for l_, _lg in whole_ends.Ends(st1).lanes]
check(st0['smenu'].get('N0') and st0['dmenu'].get('N0') and lanes0 == ['N0'],
      f'10: N0 has escapes at both arrays and a lane: {lanes0}')
check(not st1['smenu'].get('N0') and lanes1 == [], f'10: N0 with no tooth escape has no lane: {lanes1}')

# 12. the escape fanout's via reach: its moves are laid exactly as they are, so a via keeps another lane's run off by
# the rule (rules.fan_via_clear), not the router's grid ring (fan_via_reach) -- at 0.45 mm vias, 0.12 escapes and 0.1
# clearance the ring is past the 0.4 a dog-bone stands off the next ball line, and every dog-bone met every
# neighbouring line's run (sdi_mipi_conv's SDI bus: no plan of its teeth with no conflict, on four layers or eight)
r10 = _rules.Rules(clearance=0.1, track=0.15, via_size=0.45, via_drill=0.2, fan_track=0.12, source='test')
ring, clear = _rules.fan_via_reach(r10), _rules.fan_via_clear(r10)
check(clear < 0.4 < ring, f'12: at 0.45/0.12/0.1 the grid ring {ring:.5f} is past 0.4 and the rule {clear:.3f} short of it')
# (A along the ball line y 0 on B.Cu past B's dog-bone via, 0.4 off it; B a STREET move on In1.Cu, which buckets every
# move by a via's reach -- pages_first._conflicts)
ra = em.Move('A', 'dogbone', 'left', 'B.Cu', (-1.2, 0.0), 1,
             [((2.4, 0.0), (2.8, 0.4), 'F.Cu'), ((2.8, 0.4), (2.8, 0.0), 'B.Cu'), ((2.8, 0.0), (-1.2, 0.0), 'B.Cu')],
             site=(2.8, 0.4), climb=1)
rb = em.Move('B', 'dogbone', 'up', 'In1.Cu', (1.2, -1.2), 1,
             [((0.8, 0.8), (1.2, 0.4), 'F.Cu'), ((1.2, 0.4), (1.2, -1.2), 'In1.Cu')], site=(1.2, 0.4), climb=1)
rb.street = 1
check(sm._conflict(ra, rb, strict=True, stack=True, xing=2, reach=ring),
      '12: under the grid ring, A\'s run beside B\'s via is a conflict')
check(not sm._conflict(ra, rb, strict=True, stack=True, xing=2, reach=clear),
      '12: under the rule it is none')
check(not pf._conflicts({'A': [ra], 'B': [rb]}, True, stack=True, xing=2, via_reach=clear),
      '12: pages_first._conflicts passes the reach through (a street move among them)')
check(pf._conflicts({'A': [ra], 'B': [rb]}, True, stack=True, xing=2, via_reach=ring) != [],
      '12: ...and the ring\'s conflict with it')

# 13. dogbone_arrays.beneath: a passive's PASTE-only pads are no pads of its (sdi_mipi_conv's C105 under U11, two
# copper pads and two paste openings drawn as pads, read as no two-pad part and held -- no dog-bone for two SDI balls)
cap = '''  (footprint "C0402" (layer "B.Cu") (at 1.6 1.6)
    (property "Reference" "C1" (at 0 0) (layer "B.SilkS"))
    (pad "1" smd rect (at -0.48 0) (size 0.56 0.62) (layers "B.Cu" "B.Mask") (net 3 "N2"))
    (pad "2" smd rect (at 0.48 0) (size 0.56 0.62) (layers "B.Cu" "B.Mask") (net 4 "N3"))
    (pad "" smd rect (at -0.48 0) (size 0.4 0.5) (layers "B.Paste"))
    (pad "" smd rect (at 0.48 0) (size 0.4 0.5) (layers "B.Paste"))
  )
  (footprint "C3" (layer "B.Cu") (at 1.6 2.4)
    (property "Reference" "C2" (at 0 0) (layer "B.SilkS"))
    (pad "1" smd rect (at -0.5 0) (size 0.3 0.3) (layers "B.Cu" "B.Mask") (net 3 "N2"))
    (pad "2" smd rect (at 0 0) (size 0.3 0.3) (layers "B.Cu" "B.Mask") (net 4 "N3"))
    (pad "3" smd rect (at 0.5 0) (size 0.3 0.3) (layers "B.Cu" "B.Mask") (net 5 "N4"))
  )'''
pcb_c, _p = parse(board(cap))
check(dga.beneath(pcb_c, pcb_c.footprints['C1'], ['U1', 'U2']),
      '13: a two-pad cap with paste-only pads beneath U1 is beneath it (moved by the cap step, no via blocked)')
check(not dga.beneath(pcb_c, pcb_c.footprints['C2'], ['U1', 'U2']), '13: a three-pad part is not')

# 14. a lane BARE at both arrays (braid.bare_lane): no track of its net, its pads on two parts -- the outer balls the
# escape field leaves to leave on their own layer; the escape fanout lays both ends from the pads, so the ladder takes
# it (make_bench.write_ladder, coherent_nets with bare_ok) where a stub end was all it read (sdi_mipi_conv's SDI bus
# lost D0, D2 and D7 so; a synth bus of outer balls alone lost every net)
import braid as _bd  # noqa: E402
import make_bench as _mb  # noqa: E402
import coherent_nets as _cn  # noqa: E402
pcb0, path0 = parse(board())
bn0 = {n.name.split('/')[-1]: (i, n) for i, n in pcb0.nets.items()}
check(_bd.bare_lane(pcb0, 'N0', bn0), '14: N0 (U1 and U2, no copper) is a bare lane')
pcb_s, _p = parse(board('  (segment (start 0 0) (end 0.4 0.4) (width 0.12) (layer "F.Cu") (net 1))'))
check(not _bd.bare_lane(pcb_s, 'N0', {n.name.split('/')[-1]: (i, n) for i, n in pcb_s.nets.items()}),
      '14: with a stub it is not')
with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
    _mb.write_ladder(path0, ['N0'], log=lambda *a: None)
    lad_off = _cn.coherent_nets(10, path0, bare_ok=False)
    _mb.write_ladder(path0, ['N0'], log=lambda *a: None, bare_ok=True)
    lad_on = _cn.coherent_nets(10, path0, bare_ok=True)
check(lad_off == [] and lad_on == ['N0'], f'14: the ladder takes the bare lane with bare_ok alone: {lad_off} {lad_on}')

# 15. ...and the ends model prices an escape state's conflicts by it: its conflicts do not move with the grid ring
# (select_moves._VIA_REACH), which on these menus moves the conflicts priced by the ring
# (the dog-boned board, its inner 3 x 3 balls' nets on a 3 x 3 array U3 east of it)
idx15 = [i for i in range(1, N * N) if i % N in (1, 2, 3) and i // N in (1, 2, 3)]
names15 = [f'N{i}' for i in idx15]
u3 = '  (footprint "BGA3" (layer "F.Cu") (at 25 0.8)\n    (property "Reference" "U3" (at 0 0) (layer "F.SilkS"))\n' + \
    '\n'.join(f'    (pad "{k}" smd circle (at {(k % 3) * PITCH:.3f} {(k // 3) * PITCH:.3f}) (size 0.4 0.4) '
              f'(layers "F.Cu" "F.Mask") (net {i + 1} "N{i}"))' for k, i in enumerate(idx15)) + '\n  )'
pcb15, _p = parse(board(vtxt + '\n' + stxt + '\n' + u3))
with contextlib.redirect_stdout(io.StringIO()):
    st15 = ep.plan_state(pcb15, names15, 'U1', 'U3', log=lambda *a: None)
    st15['launch'] = {n: v for n, v in st15['launch'].items() if st15['smenu'].get(n) and st15['dmenu'].get(n)}
ring0 = sm._VIA_REACH
nconf, nring = [], []
try:
    for R_ in (0.30, 0.45):
        sm._VIA_REACH = R_
        with contextlib.redirect_stdout(io.StringIO()):
            E15 = whole_ends.Ends(st15)
        nconf.append(sum(len(v) for v in E15.conf.values()))
        nring.append(len(pf._conflicts({n: st15['smenu'][n] for n in st15['launch']}, True, stack=True)))
finally:
    sm._VIA_REACH = ring0
check(nring[0] != nring[1], f'15: the ring moves the conflicts it prices on these menus: {nring}')
check(nconf[0] == nconf[1] > 0, f'15: the ends model\'s conflicts on the escape state do not move with it: {nconf}')

print(f'escape plan: {len(moves)} moves from the test ball; ' + ('FAIL' if fails else 'all checks pass'))
sys.exit(1 if fails else 0)
