#!/usr/bin/env python3
"""A bus net the run leaves out is still its arrays' to plan (route_bus.others_left_out, escapes_kept).

  python3 tests/test_622_route_bus_left_out.py

sdi_mipi_conv's DDR3 bus: the source fanout dropped DDR3_A2's ball (U11.L11), so the net was in no ladder and no run --
and, a bus net, not among the array's other nets either: no round's joint plan planned its ball, the step put it back
as it came (none), and route.py found no way out of the field the bus left it.

1. others_left_out puts each net of the bus the run leaves out among the others of every array it has a ball of, and
   no net the run routes, no net of a zone (a plane's balls are the drops'), and none an array has no ball of;
2. escapes_kept keeps the joint plan's escapes of such a net only where the board as it came had no copper of its own
   on it: a net of its own copper is put back as it came, and a net the joint plan never took is put back too.
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
    print(('ok  ' if cond else 'FAIL') + ' ' + msg)
    if not cond:
        fails.append(msg)


def pad(ref, num, nid, name):
    return types.SimpleNamespace(component_ref=ref, pad_number=num, net_id=nid, net_name=name)


# U11 the source, U12 the destination; A0 and A1 routed by the run, A2 dropped at the source, A3 beyond --k with no
# ball on U12 (a fly-by net that ends on the termination), VTT a plane's net
NETS = {1: '/DDR3/DDR3_A0', 2: '/DDR3/DDR3_A1', 3: '/DDR3/DDR3_A2', 4: '/DDR3/DDR3_A3', 5: '/VTT', 6: '/OTHER'}
PADS = [pad('U11', 'L1', 1, NETS[1]), pad('U12', 'M1', 1, NETS[1]),
        pad('U11', 'L2', 2, NETS[2]), pad('U12', 'M2', 2, NETS[2]),
        pad('U11', 'L11', 3, NETS[3]), pad('U12', 'M3', 3, NETS[3]),
        pad('U11', 'L4', 4, NETS[4]),
        pad('U11', 'L5', 5, NETS[5]), pad('U12', 'M5', 5, NETS[5]),
        pad('U11', 'L6', 6, NETS[6])]
fps = {}
for p in PADS:
    fps.setdefault(p.component_ref, types.SimpleNamespace(pads=[])).pads.append(p)
nets = {i: types.SimpleNamespace(net_id=i, name=n) for i, n in NETS.items()}
pcb = types.SimpleNamespace(nets=nets, footprints=fps, zones=[types.SimpleNamespace(net_id=5)], segments=[], vias=[])
names = [NETS[1], NETS[2], NETS[3], NETS[4], NETS[5]]

print('1. others_left_out: the bus nets the run leaves out, among each array\'s others where it has a ball')
spec = {'arrays': [{'ref': 'U11', 'others': [NETS[6]], 'drops': [NETS[5]]}, {'ref': 'U12', 'others': [], 'drops': []}]}
got = rb.others_left_out(spec, pcb, names, ['DDR3_A0', 'DDR3_A1'])
check(got == [('U11', NETS[3]), ('U11', NETS[4]), ('U12', NETS[3])], f'added {got}')
check(spec['arrays'][0]['others'] == sorted([NETS[3], NETS[4], NETS[6]]), f'U11 others {spec["arrays"][0]["others"]}')
check(spec['arrays'][1]['others'] == [NETS[3]], f'U12 others (A3 has no ball there) {spec["arrays"][1]["others"]}')
check(NETS[5] not in spec['arrays'][0]['others'], 'the plane\'s net stays with the drops')
check(rb.others_left_out(spec, pcb, names, ['DDR3_A0', 'DDR3_A1']) == [], 'a second call adds nothing')
spec2 = {'arrays': [{'ref': 'U11', 'others': [], 'drops': []}]}
check(rb.others_left_out(spec2, pcb, names[:3], ['DDR3_A0', 'DDR3_A1', 'DDR3_A2']) == [], 'the whole bus run: none')

print('2. escapes_kept: a left-out net\'s escapes kept where the board had no copper of its own on it')
left = {NETS[3], NETS[4]}
back = [NETS[3], NETS[4], NETS[2]]
check(rb.escapes_kept(pcb, back, left) == [NETS[3], NETS[4]], f'no copper of their own: {rb.escapes_kept(pcb, back, left)}')
pcb.segments = [types.SimpleNamespace(net_id=4)]
check(rb.escapes_kept(pcb, back, left) == [NETS[3]], f'A3 had a track: put back {rb.escapes_kept(pcb, back, left)}')
pcb.segments, pcb.vias = [], [types.SimpleNamespace(net_id=3)]
check(rb.escapes_kept(pcb, back, left) == [NETS[4]], f'A2 had a via: put back {rb.escapes_kept(pcb, back, left)}')
check(rb.escapes_kept(pcb, back, set()) == [], 'none the joint plan took: every one put back')

if fails:
    print(f'FAIL: {len(fails)} check(s)')
    sys.exit(1)
print('PASS: a bus net the run leaves out is planned with its arrays\' other nets, its escapes kept unless it had copper')
