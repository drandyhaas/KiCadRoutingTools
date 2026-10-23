#!/usr/bin/env python3
"""The buses of one board routed in turn, each with its own fanout (route_bus --bus, route_buses).

  python3 tests/test_622_route_buses_shared.py

With the step itself stood in for (no routing: each bus 'handed on' as its input), this pins:

1. parse_bus: SRC:DEST[:MODE[:LAYERS]] -- MODE joint, escape or plain, LAYERS the bus's ROUTE_LAYERS; anything else
   named;
2. route_buses runs the buses in the order given, each with its own fanout (joint_fanout / escape) and its own
   routing layers (settings ROUTE_LAYERS), the other buses none;
3. a SHARED ARRAY: an escape bus on an array an earlier bus was handed on through gets that array planned whole
   (ESCAPE_JOINT) and its ends on the board as it was before that earlier bus (ESCAPE_JOINT_ENDS); an escape bus on
   arrays nothing went through, and one after an earlier bus that was not handed on, get neither; a plain or joint
   bus from such an array gets neither either, and no SOURCE_SHARED set on it;
3b. keeps_source_escapes: a PLAIN bus keeps the escapes an earlier step laid its nets at its source by default, a joint
   or an escape bus does not; SOURCE_SHARED=0 keeps none; SOURCE_SHARED=REF,.. a plain or joint bus from those
   sources, never an escape bus;
4. step_policy lays a step's settings over the environment, the step's own policy over both.
"""
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import route_bus as rb  # noqa: E402

fails = []


def check(cond, msg):
    if not cond:
        fails.append(msg)
        print('FAIL', msg)


# 1. parse_bus
check(rb.parse_bus('U1:U2') == ('U1', 'U2', None, None), '1: SRC:DEST')
check(rb.parse_bus('U1:U5:escape:F.Cu,B.Cu,In1.Cu') == ('U1', 'U5', 'escape', 'F.Cu,B.Cu,In1.Cu'),
      '1: SRC:DEST:MODE:LAYERS')
for bad in ('U1', 'U1:U2:fast', ':U2', 'U1:U2:joint:F.Cu:x'):
    try:
        rb.parse_bus(bad)
        check(False, f'1: {bad!r} refused')
    except ValueError:
        pass


# 2 and 3. route_buses with the step stood in for
def run(buses, fail=()):
    calls = []

    def step(board, out, src, dest, *a, joint_fanout=False, escape=False, settings=None, **kw):
        calls.append(dict(board=board, src=src, dest=dest, joint=joint_fanout, escape=escape,
                          settings=dict(settings or {})))
        shutil.copy(board, out)
        shutil.copy(board[:-len('.kicad_pcb')] + '.kicad_pro', out[:-len('.kicad_pcb')] + '.kicad_pro')
        return (1 if (src, dest) in fail else 0), f'BUS {src}->{dest}'
    real = rb.route_bus
    rb.route_bus = step
    try:
        with tempfile.TemporaryDirectory() as td:
            b0 = os.path.join(td, 'b.kicad_pcb')
            open(b0, 'w').write('(kicad_pcb)\n')
            open(os.path.join(td, 'b.kicad_pro'), 'w').write('{}\n')
            code, _g = rb.route_buses(b0, os.path.join(td, 'o.kicad_pcb'), buses, log=lambda *a: None)
            return code, calls, b0, os.path.join(td, 'o.bus')
    finally:
        rb.route_bus = real


L4 = 'F.Cu,B.Cu,In1.Cu,In2.Cu'
code, calls, b0, work = run([('U1', 'U2', 'joint', None), ('U1', 'U5', 'escape', L4), ('U7', 'U8', 'escape', None)])
check(code == 0 and [(c['src'], c['dest']) for c in calls] == [('U1', 'U2'), ('U1', 'U5'), ('U7', 'U8')],
      f'2: the buses in the order given: {[(c["src"], c["dest"]) for c in calls]}')
check(calls[0]['joint'] and not calls[0]['escape'] and calls[0]['settings'] == {}, f'2: U1-U2 joint: {calls[0]}')
check(calls[1]['escape'] and not calls[1]['joint'], f'2: U1-U5 escape: {calls[1]}')
check(calls[1]['board'] == os.path.join(work, 'U1-U2.kicad_pcb'), '2: U1-U5 on the board U1-U2 handed on')
check(calls[1]['settings'] == {'ROUTE_LAYERS': L4, 'ESCAPE_JOINT': 'U1', 'ESCAPE_JOINT_ENDS': b0},
      f'3: U1-U5: its layers, U1 shared, its ends on the board before U1-U2: {calls[1]["settings"]}')
check(calls[2]['escape'] and calls[2]['settings'] == {}, f'3: U7-U8: nothing shared: {calls[2]["settings"]}')
_c, calls, _b, _w = run([('U1', 'U2', 'joint', None), ('U1', 'U5', 'escape', None)], fail={('U1', 'U2')})
check('ESCAPE_JOINT' not in calls[1]['settings'], f'3: after a bus left as it came, nothing shared: {calls[1]}')
_c, calls, _b, _w = run([('U1', 'U2', 'joint', None), ('U1', 'U5', 'joint', None)])
check(calls[1]['settings'] == {}, f'3: a joint bus on a shared array: no ESCAPE_JOINT, no SOURCE_SHARED: {calls[1]}')
# (the zynq's four: the DDR3, the LVDS round it, then its USB and Ethernet buses to their QFNs)
_c, calls, _b, _w = run([('U1', 'U2', 'joint', None), ('U1', 'U5', 'escape', L4), ('U1', 'U4', 'plain', L4),
                         ('U1', 'U3', 'plain', L4), ('U7', 'U8', 'plain', None)])
check([c['settings'].get('SOURCE_SHARED') for c in calls] == [None] * 5,
      f'3: SOURCE_SHARED is the environment\'s to opt into, set on no bus: {[c["settings"] for c in calls]}')
check(calls[2]['settings'] == {'ROUTE_LAYERS': L4} and not calls[2]['escape'] and not calls[2]['joint'],
      f'3: U1-U4: its layers, plain: {calls[2]}')

# 3b. keeps_source_escapes (src, escape, joint_fanout, SOURCE_SHARED)
ks = rb.keeps_source_escapes
check(ks('U1', False, False, None) and ks('U1', False, False, '  '), '3b: a plain bus keeps them by default')
check(not ks('U1', False, True, None), '3b: a joint bus does not by default')
check(not ks('U1', True, False, None) and not ks('U1', True, False, 'U1'), '3b: an escape bus never')
check(not ks('U1', False, False, '0'), '3b: SOURCE_SHARED=0 keeps none')
check(ks('U1', False, True, 'U7,U1') and not ks('U1', False, False, 'U7'),
      '3b: SOURCE_SHARED=REF,..: a plain or joint bus from those sources alone')

# 4. step_policy: the step's settings over the environment, its policy over both
import awx_settings  # noqa: E402
os.environ['ROUTE_LAYERS'] = 'F.Cu,B.Cu'
with rb.step_policy(None, {'ROUTE_LAYERS': L4, 'TAUT_MEMO': '1'}):
    check(awx_settings.get('ROUTE_LAYERS') == L4, '4: the step\'s ROUTE_LAYERS over the environment\'s')
    check(awx_settings.get('TAUT_MEMO') == '0', '4: the step\'s policy (caches off) over its settings')
with rb.step_policy(None):
    check(awx_settings.get('ROUTE_LAYERS') == 'F.Cu,B.Cu', '4: no settings: the environment\'s')

print('route_buses: ' + ('FAIL' if fails else 'PASS: the buses in order, each its own fanout and layers; '
                         'a later escape bus on an array an earlier one went through plans it whole'))
sys.exit(1 if fails else 0)
