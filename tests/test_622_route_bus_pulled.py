#!/usr/bin/env python3
"""A bus net with OTHER PADS beside its two ends (route_bus.pulled_nets, unpull, repull).

  python3 tests/test_622_route_bus_pulled.py

1. pulled_nets: a net with one pad on each of the bus's two parts and any other pads is pulled, each other pad a stub
   -- a pull to a plane, a series part's, a pad on a part carrying other signals, a test point's; one already among the
   bus's nets, one with two pads on an end, and a rail -- a plane's own net (one that owns a zone), a net on more parts
   than LEFT_PARTS -- are not;
1b. a fly-by bus's terminations: the nets on a resistor ARRAY to a Vtt, a pair whose legs a part joins (a clock pair's
   termination) and two nets a part joins that are no pair are pulled, and still are once the Vtt is on few parts;
2. unpull takes the pull pad off its net (its (net ...) token, and nothing else) and returns the token; repull puts it
   back after the pad's (layers ...), the text as it was; a pad not found is named.
3. _came_with: copper put back as it came stands on a pull pad the step reads with no net -- a via of the net in its
   own termination's pad. The pad is given the copper's net where the copper stood on it on the board it is taken
   from, the pad where it was and of no other net there; not a pad that moved, one of another net there, one the
   copper is clear of, or a track's on another layer.
4. The step's two clearance grades read such a pad as the net's: true_view writes the board with the held pads back
   on their nets (the text as it was), and whole_route.drc_count leaves out a row of a net's copper against its own
   pull pad -- not one against another net's pull pad, an accepted row, or one of a pad that is no pull pad.
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


NETS = {1: 'GND', 2: 'PULLED', 3: 'BUS', 4: 'SERIES', 5: 'MID', 6: 'MULTI', 7: 'TWICE', 8: 'OTHER', 9: 'TP'}
PADS = [pad('U1', 'A1', 2), pad('U5', 'B1', 2), pad('RA', '2', 2), pad('RA', '1', 1),       # PULLED: RA to GND
        pad('U1', 'A2', 3), pad('U5', 'B2', 3), pad('RB', '2', 3), pad('RB', '1', 1),       # BUS: in names already
        pad('U1', 'A3', 4), pad('U5', 'B3', 4), pad('RC', '2', 4), pad('RC', '1', 5),       # SERIES: RC to a signal
        pad('U1', 'A4', 6), pad('U5', 'B4', 6), pad('U9', '1', 6), pad('U9', '2', 1),       # MULTI: U9 not two-pad
        pad('U9', '3', 8), pad('U9', '4', 8),
        pad('U1', 'A5', 7), pad('U1', 'A6', 7), pad('U5', 'B5', 7), pad('RD', '2', 7), pad('RD', '1', 1),  # TWICE
        pad('U1', 'A7', 1), pad('U5', 'B7', 1),                                              # GND: the plane's own
        pad('U1', 'A8', 9), pad('U5', 'B8', 9), pad('TP1', '1', 9)]                          # TP: a test point
nets = {i: types.SimpleNamespace(net_id=i, name=n, pads=[p for p in PADS if p.net_id == i]) for i, n in NETS.items()}
fps = {}
for p in PADS:
    fps.setdefault(p.component_ref, types.SimpleNamespace(pads=[])).pads.append(p)
pcb = types.SimpleNamespace(nets=nets, footprints=fps, zones=[types.SimpleNamespace(net_id=1)])
got = rb.pulled_nets(pcb, 'U1', 'U5', ['BUS'])
check(sorted(got) == ['MULTI', 'PULLED', 'SERIES', 'TP'], f'1: PULLED, SERIES, MULTI and TP: {sorted(got)}')
check([(p.component_ref, p.pad_number) for p in got.get('PULLED', ())] == [('RA', '2')], f'1: its pull pad: {got}')
check([(p.component_ref, p.pad_number) for p in got.get('TP', ())] == [('TP1', '1')], f'1: the test point: {got}')
check([(p.component_ref, p.pad_number) for p in got.get('MULTI', ())] == [('U9', '1')], f'1: MULTI\'s stub: {got}')
for k in range(rb.LEFT_PARTS):          # (TP's net on more parts than LEFT_PARTS: a rail)
    tp_ = pad(f'TQ{k}', '1', 9)
    nets[9].pads.append(tp_)
    fps[tp_.component_ref] = types.SimpleNamespace(pads=[tp_])
check('TP' not in rb.pulled_nets(pcb, 'U1', 'U5', ['BUS']), '1: a net on more parts than LEFT_PARTS is a rail')

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
check(sorted(got) == ['A0', 'A1', 'CK_N', 'CK_P', 'X0', 'X1'], f'1b: the array\'s, the pair\'s across RT, RX\'s: '
      f'{sorted(got)}')
check([(p.component_ref, p.pad_number) for p in got.get('A1', ())] == [('RN', '2')], f'1b: A1\'s stub pad: {got}')
for p in [p for p in PADS2 if p.component_ref.startswith('C')]:
    fps2.pop(p.component_ref, None)
    nets2[1].pads.remove(p)
check(sorted(rb.pulled_nets(pcb2, 'U1', 'U5', [])) == ['A0', 'A1', 'CK_N', 'CK_P', 'X0', 'X1'],
      '1b: the Vtt on few parts, the array\'s nets pulled still')

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

# 3. _came_with: RA.2 (B.Cu, 0.4 x 0.5 at (5, 5)) off its net in the step's view; a via of PULLED at (4.9, 5)
sys.path.insert(0, os.path.join(ROOT, 'tests'))
from synth import make_pad  # noqa: E402

CLEAR = 0.09


def board_(net_id, x=5.0, name='PULLED'):
    """a board of one part RA whose pad 2 is at (x, 5) on B.Cu, on net `net_id` (0: off its net)"""
    p = make_pad(net_id, x, 5.0, ref='RA', num='2', net_name=name if net_id else '', size_x=0.4, size_y=0.5,
                 layers=['B.Cu', 'B.Mask', 'B.Paste'])
    return types.SimpleNamespace(footprints={'RA': types.SimpleNamespace(pads=[p])},
                                 nets={2: types.SimpleNamespace(name='PULLED'), 7: types.SimpleNamespace(name='OTHER')})


def came_with(came, read, now=None):
    now = now or board_(0)
    rb._came_with(now, came, read, CLEAR)
    return now.footprints['RA'].pads[0].net_id


LAYERS = ['F.Cu', 'In1.Cu', 'B.Cu']
VIA = [('via', 2, [(0.25, (4.9, 5.0), (4.9, 5.0), L) for L in LAYERS])]
check(came_with(board_(2), VIA) == 2, '3: a via of the net in its own pad on the board it came from: the pad its net')
check(came_with(board_(0), VIA) == 2, '3: the pad off its net on that board too (a step\'s own bench): its net')
check(came_with(None, VIA) == 0, '3: no board it came from: nothing given')
check(came_with(board_(7, name='OTHER'), VIA) == 0, '3: another net\'s pad on the board it came from: not given')
check(came_with(board_(2, x=5.3), VIA) == 0, '3: a pad that moved since: not given')
FAR = [('via', 2, [(0.25, (5.6, 5.0), (5.6, 5.0), L) for L in LAYERS])]
check(came_with(board_(2), FAR) == 0, '3: a via clear of the pad: not given')
NEAR = [('via', 2, [(0.25, (5.4, 5.0), (5.4, 5.0), L) for L in LAYERS])]
check(came_with(board_(2), NEAR) == 2, '3: a via within the clearance of the pad, as it came: its net')
check(came_with(board_(2), [('seg', 2, [(0.1, (4.0, 5.0), (5.0, 5.0), 'F.Cu')])]) == 0,
      '3: a track over the pad on another layer: not given')
check(came_with(board_(2), [('seg', 2, [(0.1, (4.0, 5.0), (5.0, 5.0), 'B.Cu')])]) == 2,
      '3: a track ending on the pad on its layer: its net')
check(came_with(board_(2), [('via', 0, VIA[0][2])]) == 0, '3: copper of no net: not given')

# 4. the grades: the board as it is, and a lay's rows less the net's own pull pad's
import json  # noqa: E402
import tempfile  # noqa: E402
import whole_route as wr  # noqa: E402

with tempfile.TemporaryDirectory() as tmp:
    view, true_ = os.path.join(tmp, 'view.kicad_pcb'), os.path.join(tmp, 'true.kicad_pcb')
    with open(view, 'w', encoding='utf-8') as f:
        f.write(t2)
    check(rb.true_view(view, [], true_) == view, '4: no pad held: the board itself')
    got_ = rb.true_view(view, held, true_)
    check(got_ == true_ and open(true_, encoding='utf-8').read() == TXT, '4: the held pad back on its net, the text as it was')
    rows = [dict(type='pad-via', pad_ref='RA.2', net1='net_0', net2='/sheet/PULLED'),          # its own pull pad
            dict(type='pad-hole', pad_ref='RA.2', net1='PULLED', net2='net_0'),                 # ...and its hole's row
            dict(type='pad-via', pad_ref='RA.2', net1='net_0', net2='OTHER'),                   # another net on it
            dict(type='pad-via', pad_ref='RB.1', net1='net_0', net2='PULLED'),                  # no pull pad
            dict(type='via-in-paste', pad_ref='RA.2', net1='PULLED', accepted='protected')]     # accepted: not counted
    jp = os.path.join(tmp, 'drc.json')
    with open(jp, 'w', encoding='utf-8') as f:
        json.dump(dict(violations=4, by_type={'pad-via': 3, 'pad-hole': 1}, items=rows), f)
    check(wr.drc_count(jp)[0] == 4, f'4: no pull pads given: every row counted ({wr.drc_count(jp)[0]})')
    check(wr.drc_count(jp, {'RA.2': 'PULLED'})[0] == 2,
          f'4: the two rows of the net on its own pull pad left out ({wr.drc_count(jp, {"RA.2": "PULLED"})[0]})')

print('route_bus pulled: ' + ('FAIL' if fails else 'PASS: the nets with other pads found; a stub pad off its net '
                                                  'and back, the text as it was'))
sys.exit(1 if fails else 0)
