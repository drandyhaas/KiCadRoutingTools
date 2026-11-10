#!/usr/bin/env python3
"""The whole-array plan chooses each run's LAYER itself when a colouring leaves a bus ball's run out
(awx/joint_escape.py: plan_array's `exact_layers`).

  python3 tests/test_622_run_layers.py

On more than two routing layers a via escape's run is planned on no one layer (RUN): the model's capacity rows count
the runs of each lane, crossing and exit cluster against the layers they may take, and colour_runs gives each chosen
run its layer afterwards. A run through many such groups can meet every count and still find each of its layers taken
by one neighbour or another -- a shared array's inner balls, their runs out across twenty-three others on seven
layers: no colouring held them. The plan was then asked again with those runs' options out of its menus, the next
plan's runs met the same wall elsewhere, and after three plans a bus ball was left with no escape. Asked again with
`exact_layers` instead, the model carries a literal per run and layer -- no two runs of a group on one, a pair's legs
on one -- so a plan it returns has a layer for every run.

On kicad_files/ulx3s.kicad_pcb U1, a block of its inner balls the bus's, four routing layers, via escapes only:
1. LIVENESS: the plan's chosen runs include runs that must stand on different layers (colouring edges); else the
   checks below test nothing;
2. `exact_layers`, the colouring stood in for to give NO run a layer (as when its time runs out): every run has the
   layer the plan chose it, no two runs joined by an edge on one, and every ball the plain plan serves is served;
3. control: the same stand-in without `exact_layers` leaves the runs with no layer -- the layers of 2 are the model's;
4. the retry: a colouring that leaves a bus ball's run out asks the plan again with `exact_layers` and nothing banned;
   with JOINT_LAYERS_EXACT off, with that run's option banned, as before.
"""
import contextlib
import io
import os
import sys

try:
    import ortools  # noqa: F401
except ImportError as exc:
    print('SKIP: needs ortools (%s)' % exc)
    sys.exit(77)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.chdir(os.path.join(ROOT, 'awx'))
BOARD = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')
if not os.path.exists(BOARD):
    print(f'SKIP: board not present: {BOARD}')
    sys.exit(77)
os.environ['ROUTE_LAYERS'] = 'F.Cu,B.Cu,In1.Cu,In2.Cu'
os.environ.pop('FANOUT_JOINT', None)

import joint_escape as je  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from coherent_nets import namer  # noqa: E402

BAD = []
N = [0]


def check(ok, what):
    N[0] += 1
    print(('PASS: ' if ok else 'FAIL: ') + what)
    if not ok:
        BAD.append(what)


with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
    pcb = parse_kicad_pcb(BOARD)
foot = pcb.footprints['U1']
sn = namer(pcb)
zone_ids = {z.net_id for z in (pcb.zones or []) if z.net_id}
ys = sorted({round(p.global_y, 3) for p in foot.pads if p.net_id})
xs = sorted({round(p.global_x, 3) for p in foot.pads if p.net_id})
block = [p for p in foot.pads if p.net_id and p.net_id not in zone_ids and len(pcb.nets[p.net_id].pads) > 1
         and not sn(p.net_name).startswith(('GND', '+', 'VCC'))
         and round(p.global_y, 3) in ys[2:5] and round(p.global_x, 3) in xs[2:9]]
bus = sorted({p.net_name for p in block})
at_of = {f'{sn(p.net_name)}#{p.pad_number}': (round(p.global_x, 3), round(p.global_y, 3)) for p in block}
layers = list(pcb.board_info.copper_layers)
colour0, plan0 = je.colour_runs, je.plan_array


def plan(colour=None, **kw):
    """(hints, report, the colour_runs calls' (runs, edges), plan_array's calls' (exact_layers, bans))"""
    seen, asked = [], []

    def colour_(runs, edges, together, want, kinds=None):
        seen.append((dict(runs), list(edges)))
        return (colour or colour0)(runs, edges, together, want, kinds)

    def plan_(*a, **k):
        asked.append((bool(k.get('exact_layers')), len(k.get('ban') or ())))
        return plan0(*a, **k)
    je._SOLVED.clear()
    je._LAST_PLAN.clear()
    je.colour_runs, je.plan_array = colour_, plan_
    try:
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            hints, rep = je.plan_array(pcb, 'U1', bus, [], layers, log=lambda *a: None, vias_only=True,
                                       stack_bus=True, **kw)
    finally:
        je.colour_runs, je.plan_array = colour0, plan0
    return hints, rep, seen, asked


def run_layer(hints, key):
    """the layer the ball's run stands on in the plan's hints: its legs' one inner or back layer, None if it has none"""
    h = hints.get(at_of[key]) or {}
    on = {L for *_ab, L in h.get('legs') or () if L != 'F.Cu'}
    return next(iter(on)) if len(on) == 1 else None


none = lambda runs, edges, together, want, kinds=None: ({}, sorted(runs))     # noqa: E731

# ---- 1, 2. exact_layers, the colouring giving nothing
plain_h, plain_r, _s, _a = plan()
hints, rep, seen, asked = plan(colour=none, exact_layers=True)
runs, edges = seen[0]
check(len(edges) >= 2 and len(runs) >= 10,
      f'liveness: {len(runs)} runs chosen, {len(edges)} pair(s) of them to stand on different layers')
lay = {k: run_layer(hints, k) for k in runs}
check(all(lay[k] in runs[k] for k in runs),
      f'exact_layers, the colouring giving none a layer: every run has one of its own layers ({sorted(set(lay.values()), key=str)})')
check(all(lay[a] != lay[b] for a, b in edges) and not rep.get('uncoloured'),
      'no two runs that must differ stand on one layer, none reported uncoloured')
check(rep['bus_escaped'] >= plain_r['bus_escaped'] and len(rep['unplanned']) <= len(plain_r['unplanned']),
      f'every ball the plain plan serves is served ({rep["bus_escaped"]} of {rep["bus_balls"]}, plain '
      f'{plain_r["bus_escaped"]})')

# ---- 3. control: without the layer literals the stand-in leaves the runs out
exact0 = je.LAYERS_EXACT
je.LAYERS_EXACT = False                 # (and no plan asked again: colour_tries 0)
try:
    h3, r3, seen3, _a3 = plan(colour=none, colour_tries=0)
finally:
    je.LAYERS_EXACT = exact0
check(len(r3.get('uncoloured') or ()) == len(seen3[0][0]) > 0,
      f'control: the same stand-in without exact_layers leaves {len(r3.get("uncoloured") or ())} run(s) no layer')

# ---- 4. the retry
state = {'n': 0}


def first_out(runs, edges, together, want, kinds=None):
    col, unc = colour0(runs, edges, together, want, kinds)
    state['n'] += 1
    if state['n'] == 1:
        k = sorted(runs)[0]
        col = {x: L for x, L in col.items() if x != k}
        unc = sorted(set(unc) | {k})
    return col, unc


for on in (True, False):
    state['n'] = 0
    exact0 = je.LAYERS_EXACT
    je.LAYERS_EXACT = on
    try:
        h4, r4, _s4, asked4 = plan(colour=first_out)
    finally:
        je.LAYERS_EXACT = exact0
    want = [(False, 0), (True, 0)] if on else [(False, 0), (False, 1)]
    check(asked4 == want and not r4.get('uncoloured'),
          ('a run left out: the plan asked again with each run\'s layer its own, nothing banned'
           if on else 'JOINT_LAYERS_EXACT off: asked again with that run\'s option banned, as before')
          + f' (plan_array asked {asked4})')

print(f'\n{N[0] - len(BAD)}/{N[0]} checks passed')
sys.exit(1 if BAD else 0)
