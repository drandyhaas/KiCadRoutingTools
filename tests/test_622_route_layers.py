#!/usr/bin/env python3
"""More routing layers than two (ROUTE_LAYERS): the layers' orders, a via end's run, and what that run may move beside.

  python3 tests/test_622_route_layers.py

1. route_layers.layers: F.Cu,B.Cu by default, the inner ones after them; refused out of that order or named twice.
   stacked: the board's stack order (F.Cu, the inner ones, B.Cu) -- the order the fanout engine lays a via in, from
   its first layer to its last: handed F.Cu,B.Cu,In2.Cu as given, it laid every via F.Cu-In2.Cu.
2. escape_vias: off by default on any count of layers; 'dest' the destination's, 'both' both; anything else refused.
3. relayer.run_to_via: a stub's run from its end back to its via, and that via; None short of a via or at a fork.
4. relayer.clashes: another net's segment within the rule on a layer bans that layer, not one it is clear of; another
   net's pad on F.Cu bans F.Cu; two runs that cross part; a run's own net bans nothing; a pair's legs one run.
"""
import os
import sys
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import awx_settings  # noqa: E402
import route_layers  # noqa: E402
import relayer  # noqa: E402

BAD = []


def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        BAD.append(what)


def refused(fn):
    try:
        fn()
    except SystemExit:
        return True
    return False


STACK = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']

# 1. the layers and their orders
with awx_settings.given({}):
    check(route_layers.layers() == ('F.Cu', 'B.Cu'), 'default routing layers F.Cu,B.Cu')
    check(route_layers.stacked(STACK) == ['F.Cu', 'B.Cu'], 'two layers stacked F.Cu,B.Cu on a four-layer board')
    check(route_layers.stacked(None) == ['F.Cu', 'B.Cu'], 'no copper list: F.Cu,B.Cu')
with awx_settings.given({'ROUTE_LAYERS': 'F.Cu,B.Cu,In2.Cu'}):
    check(route_layers.layers() == ('F.Cu', 'B.Cu', 'In2.Cu'), 'three routing layers as given, F.Cu and B.Cu first')
    check(route_layers.stacked(STACK) == ['F.Cu', 'In2.Cu', 'B.Cu'],
          'stacked in the board\'s order: the inner layer between, B.Cu last (a via F.Cu to B.Cu)')
    check(route_layers.index('In2.Cu') == 2, 'an inner layer indexed after F.Cu 0 and B.Cu 1')
for bad in ('In2.Cu,F.Cu,B.Cu', 'F.Cu,B.Cu,In2.Cu,In2.Cu'):
    with awx_settings.given({'ROUTE_LAYERS': bad}):
        check(refused(route_layers.layers), f'ROUTE_LAYERS={bad} refused')

# 2. the escapes through vias
for env, src, dst in (({}, False, False), ({'ROUTE_LAYERS': 'F.Cu,B.Cu,In2.Cu'}, False, False),
                      ({'ESCAPE_VIAS': 'dest'}, False, True), ({'ESCAPE_VIAS': 'both'}, True, True)):
    with awx_settings.given(env):
        check((route_layers.escape_vias('src'), route_layers.escape_vias('dest')) == (src, dst),
              f'escape_vias under {env or "nothing set"}: source {src}, destination {dst}')
with awx_settings.given({'ESCAPE_VIAS': 'yes'}):
    check(refused(lambda: route_layers.escape_vias('dest')), 'ESCAPE_VIAS=yes refused')


# 3. a stub's run back to its via
def seg(x0, y0, x1, y1, layer, net, w=0.1):
    return NS(start_x=x0, start_y=y0, end_x=x1, end_y=y1, width=w, layer=layer, net_id=net)


def via(x, y, net, size=0.45):
    return NS(x=x, y=y, size=size, net_id=net)


def pad(x, y, net, layers=('F.Cu',), size=0.4, kind='smd'):
    return NS(global_x=x, global_y=y, size_x=size, size_y=size, layers=list(layers), pad_type=kind, net_id=net)


# net 1: a dog-bone -- its pad at (0, 0), a neck on F.Cu to its via at (0.4, 0.4), then its run on B.Cu out to (3, 0.4)
neck = seg(0, 0, 0.4, 0.4, 'F.Cu', 1)
run1 = [seg(0.4, 0.4, 1.5, 0.4, 'B.Cu', 1), seg(1.5, 0.4, 3.0, 0.4, 'B.Cu', 1)]
v1 = via(0.4, 0.4, 1)
pcb = NS(segments=[neck] + run1, vias=[v1], footprints={'U1': NS(pads=[pad(0, 0, 1)])})
got = relayer.run_to_via(pcb, 1, (3.0, 0.4), 'B.Cu', with_via=True)
check(got is not None and got[0] == run1[::-1] and got[1] is v1, 'the run from its end back to its via, and the via')
check(relayer.run_to_via(pcb, 1, (3.0, 0.4), 'B.Cu') == run1[::-1], 'without with_via: the run alone')
check(relayer.run_to_via(pcb, 1, (3.0, 0.4), 'In2.Cu') is None, 'no run on a layer it is not on')
fork = NS(segments=run1 + [seg(1.5, 0.4, 1.5, 1.5, 'B.Cu', 1)], vias=[v1], footprints={})
check(relayer.run_to_via(fork, 1, (3.0, 0.4), 'B.Cu') is None, 'a fork: None')
short = NS(segments=run1, vias=[], footprints={})
check(relayer.run_to_via(short, 1, (3.0, 0.4), 'B.Cu') is None, 'no via at its end: None')

# 4. what a run may move beside (clearance 0.09: a run and a 0.1 track clash nearer than 0.19 between centrelines)
CL = 0.09
L3 = ['F.Cu', 'B.Cu', 'In2.Cu']
runA = [seg(0, 0, 4, 0, 'B.Cu', 1)]                 # along y 0
runB = [seg(2, -1, 2, 1, 'In2.Cu', 2)]              # across it at x 2, on another layer
near_in2 = seg(0, 0.15, 1.5, 0.15, 'In2.Cu', 3)     # another net 0.15 off run A on In2.Cu: within the rule
far_f = seg(0, 0.5, 4, 0.5, 'F.Cu', 4)              # another net 0.5 off on F.Cu: clear
own_in2 = seg(0, 0.05, 1.5, 0.05, 'In2.Cu', 1)      # run A's own net beside it on In2.Cu: never a clash
pcb4 = NS(segments=runA + runB + [near_in2, far_f, own_in2], vias=[], footprints={})
ban, sep = relayer.clashes(pcb4, {'A': ({1}, runA), 'B': ({2}, runB)}, L3, CL)
check('In2.Cu' in ban.get('A', set()), 'another net within the rule on In2.Cu bans In2.Cu')
check('F.Cu' not in ban.get('A', set()), 'another net clear of the run on F.Cu bans nothing')
check(sep == [('A', 'B')], 'two runs crossing (laid on two layers) part')
check('B.Cu' not in ban.get('A', set()) and 'In2.Cu' not in ban.get('B', set()),
      'a run is never banned by the other run (they part instead) nor on its own layer by itself')
pcb8 = NS(segments=runA + [own_in2], vias=[], footprints={})
ban8, _ = relayer.clashes(pcb8, {'A': ({1}, runA)}, L3, CL)
check(not ban8, 'the run\'s own net\'s track beside it on In2.Cu bans nothing')
pcb5 = NS(segments=list(runA), vias=[], footprints={'U9': NS(pads=[pad(2.0, 0.2, 7)])})
ban5, _ = relayer.clashes(pcb5, {'A': ({1}, runA)}, L3, CL)
check(ban5.get('A') == {'F.Cu'}, 'another net\'s pad on F.Cu over the run bans F.Cu alone')
pcb6 = NS(segments=list(runA), vias=[], footprints={'U9': NS(pads=[pad(2.0, 0.2, 1)])})
ban6, _ = relayer.clashes(pcb6, {'A': ({1}, runA)}, L3, CL)
check(not ban6, 'the run\'s own net\'s pad bans nothing')
legP, legN = [seg(0, 0, 4, 0, 'B.Cu', 5)], [seg(0, 0.2, 4, 0.2, 'B.Cu', 6)]
pcb7 = NS(segments=legP + legN, vias=[], footprints={})
ban7, sep7 = relayer.clashes(pcb7, {'P': ({5, 6}, legP + legN)}, L3, CL)
check(not ban7 and not sep7, 'a pair\'s two legs, one run: neither bans the other')

print(f'\n{"PASS" if not BAD else "FAIL"}: {len(BAD)} failure(s)' + (': ' + '; '.join(BAD) if BAD else ''))
sys.exit(1 if BAD else 0)
