#!/usr/bin/env python3
"""A bus net with a PULL or a TERMINATION beside its two ends (route_bus.pulled_nets, unpull, repull).

  python3 tests/test_622_route_bus_pulled.py

1. pulled_nets: a net with one pad on each of the bus's two parts whose every other pad is a two-pad part's, the
   part's other pad on a plane's net (one that owns a zone), is pulled; one already among the bus's nets, one whose
   extra part's other pad is a signal (a series part), one with a third pad on a part carrying another signal, one
   with two pads on an end, and a plane's own net, are not;
1b. a fly-by bus's terminations: the nets on a resistor ARRAY whose other pads are a rail's -- a net on more parts than
   LEFT_PARTS, no zone needed (a Vtt) -- and a pair whose legs a part joins (a clock pair's termination) are pulled;
   two nets a part joins that are no pair are not, nor the array's once its rail is on few parts;
2. unpull takes the pull pad off its net (its (net ...) token, and nothing else) and returns the token; repull puts it
   back after the pad's (layers ...), the text as it was; a pad not found is named.
"""
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import route_bus as rb  # noqa: E402

fails = []


def check(cond, msg):
    if not cond:
        fails.append(msg)
        print('FAIL', msg)


# 1. pulled_nets, on a board stood in for: U1 and U5 the bus's parts, RA..RD two-pad parts, U9 a part of four pads
def pad(ref, num, nid):
    return types.SimpleNamespace(component_ref=ref, pad_number=num, net_id=nid)


NETS = {1: 'GND', 2: 'PULLED', 3: 'BUS', 4: 'SERIES', 5: 'MID', 6: 'MULTI', 7: 'TWICE', 8: 'OTHER'}
PADS = [pad('U1', 'A1', 2), pad('U5', 'B1', 2), pad('RA', '2', 2), pad('RA', '1', 1),       # PULLED: RA to GND
        pad('U1', 'A2', 3), pad('U5', 'B2', 3), pad('RB', '2', 3), pad('RB', '1', 1),       # BUS: in names already
        pad('U1', 'A3', 4), pad('U5', 'B3', 4), pad('RC', '2', 4), pad('RC', '1', 5),       # SERIES: RC to a signal
        pad('U1', 'A4', 6), pad('U5', 'B4', 6), pad('U9', '1', 6), pad('U9', '2', 1),       # MULTI: U9 not two-pad
        pad('U9', '3', 8), pad('U9', '4', 8),
        pad('U1', 'A5', 7), pad('U1', 'A6', 7), pad('U5', 'B5', 7), pad('RD', '2', 7), pad('RD', '1', 1),  # TWICE
        pad('U1', 'A7', 1), pad('U5', 'B7', 1)]                                              # GND: the plane's own
nets = {i: types.SimpleNamespace(net_id=i, name=n, pads=[p for p in PADS if p.net_id == i]) for i, n in NETS.items()}
fps = {}
for p in PADS:
    fps.setdefault(p.component_ref, types.SimpleNamespace(pads=[])).pads.append(p)
pcb = types.SimpleNamespace(nets=nets, footprints=fps, zones=[types.SimpleNamespace(net_id=1)])
got = rb.pulled_nets(pcb, 'U1', 'U5', ['BUS'])
check(sorted(got) == ['PULLED'], f'1: PULLED alone: {sorted(got)}')
check([(p.component_ref, p.pad_number) for p in got.get('PULLED', ())] == [('RA', '2')], f'1: its pull pad: {got}')
pcb.zones = []
check(rb.pulled_nets(pcb, 'U1', 'U5', ['BUS']) == {}, '1: no plane, no pull')

# 1b. TERMINATIONS (a fly-by DDR3 bus's address nets): RN a four-element array, two of its pads on bus-to-be nets
#     A0/A1, the others on VTT -- a rail by the parts it is on (no zone); RT across the pair CK_P/CK_N; RX across two
#     nets that are no pair, which is no termination
NETS2 = {1: 'VTT', 2: 'A0', 3: 'A1', 4: 'CK_P', 5: 'CK_N', 6: 'X0', 7: 'X1'}
PADS2 = [pad('U1', 'A1', 2), pad('U5', 'B1', 2), pad('RN', '1', 2), pad('RN', '8', 1),
         pad('U1', 'A2', 3), pad('U5', 'B2', 3), pad('RN', '2', 3), pad('RN', '7', 1),
         pad('RN', '3', 0), pad('RN', '4', 0), pad('RN', '5', 1), pad('RN', '6', 1),
         pad('U1', 'A3', 4), pad('U5', 'B3', 4), pad('RT', '1', 4),
         pad('U1', 'A4', 5), pad('U5', 'B4', 5), pad('RT', '2', 5),
         pad('U1', 'A5', 6), pad('U5', 'B5', 6), pad('RX', '1', 6),
         pad('U1', 'A6', 7), pad('U5', 'B6', 7), pad('RX', '2', 7)] + \
    [pad(f'C{k}', '1', 1) for k in range(rb.LEFT_PARTS)]             # VTT's decoupling: more parts than LEFT_PARTS
nets2 = {i: types.SimpleNamespace(net_id=i, name=n, pads=[p for p in PADS2 if p.net_id == i]) for i, n in NETS2.items()}
fps2 = {}
for p in PADS2:
    fps2.setdefault(p.component_ref, types.SimpleNamespace(pads=[])).pads.append(p)
pcb2 = types.SimpleNamespace(nets=nets2, footprints=fps2, zones=[])
got = rb.pulled_nets(pcb2, 'U1', 'U5', [])
check(sorted(got) == ['A0', 'A1', 'CK_N', 'CK_P'], f'1b: the array\'s two nets and the pair across RT: {sorted(got)}')
check([(p.component_ref, p.pad_number) for p in got.get('A1', ())] == [('RN', '2')], f'1b: A1\'s stub pad: {got}')
for p in [p for p in PADS2 if p.component_ref.startswith('C')]:
    fps2.pop(p.component_ref, None)
    nets2[1].pads.remove(p)
check(sorted(rb.pulled_nets(pcb2, 'U1', 'U5', [])) == ['CK_N', 'CK_P'], '1b: VTT on few parts and no zone, no rail')

# 2. unpull and repull on a board's text
TXT = '''(kicad_pcb
\t(footprint "R_0402"
\t\t(layer "B.Cu")
\t\t(at 1 2 90)
\t\t(property "Reference" "RA"
\t\t\t(at 0 0 0)
\t\t)
\t\t(pad "1" smd rect
\t\t\t(at -0.4 0 90)
\t\t\t(size 0.6 0.65)
\t\t\t(layers "B.Cu" "B.Mask" "B.Paste")
\t\t\t(net "GND")
\t\t\t(uuid "aaaa")
\t\t)
\t\t(pad "2" smd rect
\t\t\t(at 0.4 0 90)
\t\t\t(size 0.6 0.65)
\t\t\t(layers "B.Cu" "B.Mask" "B.Paste")
\t\t\t(net "PULLED")
\t\t\t(pinfunction "x(y)")
\t\t\t(uuid "bbbb")
\t\t)
\t)
)
'''
t2, held = rb.unpull(TXT, [pad('RA', '2', 2)])
check(held == [('RA', '2', '\n\t\t\t(net "PULLED")')], f'2: the token taken: {held}')
check(t2 == TXT.replace('\n\t\t\t(net "PULLED")', ''), '2: unpull takes the token and nothing else')
check('(net "GND")' in t2, '2: the other pad keeps its net')
t3, missed = rb.repull(t2, held)
check(t3 == TXT and missed == [], f'2: repull puts it back where it was: missed {missed}')
_t, missed = rb.repull(t2, [('RZ', '2', '(net "X")')])
check(missed == ['RZ.2'], f'2: a pad not found is named: {missed}')
t4, held4 = rb.unpull(TXT, [pad('RQ', '1', 1)])
check(t4 == TXT and held4 == [], '2: a pad not on the board: nothing taken')

print('route_bus pulled: ' + ('FAIL' if fails else 'PASS: a pulled net found; its pull pad off its net and back, '
                                                  'the text as it was'))
sys.exit(1 if fails else 0)
