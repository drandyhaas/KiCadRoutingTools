#!/usr/bin/env python3
"""The layers the joint fanout's other nets escape on only where they must (awx/joint_escape.py: plane_layers,
C_PLANE_LAYER).

  python3 tests/test_622_plane_layers.py

route_bus's joint spec offers the arrays' other nets every copper layer, and the plan prices a leg on a PLANE layer
(C_PLANE_LAYER, ten vias): an inner layer carrying a pour that the run does not route on. On the zynq DDR bench, its
In1 GND and In2 supply islands under U1: offered them at that price, U1's plan served every ball, 17 escapes going
inside, where on F.Cu and B.Cu alone eight plane balls of its outer rings were walled in.

On a hand-made board (four copper layers; pours on F.Cu, In1.Cu and In2.Cu):
1. with the run on F.Cu and B.Cu, In1.Cu and In2.Cu are plane layers, F.Cu (an outer layer's pour) is not;
2. an inner layer the run routes on (ROUTE_LAYERS) is not -- its own lanes cut that pour already;
3. a layer with no pour is not, nor a zone with no net (a keepout);
4. the price is less than serving a ball is worth, and more than a few vias.
"""
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('ROUTE_LAYERS', None)
import joint_escape as je  # noqa: E402

BAD = []


def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        BAD.append(what)


def board(zones):
    return types.SimpleNamespace(
        zones=[types.SimpleNamespace(net_id=n, layer=L) for n, L in zones],
        board_info=types.SimpleNamespace(copper_layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']))


pcb = board([(1, 'F.Cu'), (1, 'In1.Cu'), (2, 'In2.Cu')])
got = je.plane_layers(pcb)
check(got == ['In1.Cu', 'In2.Cu'], f'the run on F.Cu and B.Cu: the poured inner layers are plane layers ({got})')

os.environ['ROUTE_LAYERS'] = 'F.Cu,B.Cu,In2.Cu'
got = je.plane_layers(pcb)
check(got == ['In1.Cu'], f'an inner layer the run routes on is not ({got})')
os.environ.pop('ROUTE_LAYERS')

got = je.plane_layers(board([(1, 'In1.Cu'), (0, 'In2.Cu')]))
check(got == ['In1.Cu'], f'a keepout (a zone with no net) and a layer with no pour are not ({got})')

check(3 * je.C_VIA < je.C_PLANE_LAYER < min(je.W_OTHER, je.W_DROP),
      f'a leg on one costs more than three vias and less than serving a ball ({je.C_PLANE_LAYER} vs via '
      f'{je.C_VIA}, ball {je.W_OTHER})')

print(f'\n{"PASS" if not BAD else "FAIL"}: {len(BAD)} failure(s)' + (': ' + '; '.join(BAD) if BAD else ''))
sys.exit(1 if BAD else 0)
