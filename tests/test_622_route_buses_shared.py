#!/usr/bin/env python3
"""The buses of one board routed sequentially, each with its own fanout (route_bus --bus, route_buses).

  python3 tests/test_622_route_buses_shared.py

With the step itself stood in for (no routing: each bus 'handed on' as its input), this pins:

1. parse_bus: SRC:DEST[:MODE[:LAYERS]] -- MODE plain, whole-array or escape, LAYERS the bus's ROUTE_LAYERS; anything
   else named (the old name of whole-array, joint, among them);
2. route_buses runs the buses in the order given, each with its own fanout (whole_array / escape) and its own
   routing layers (settings ROUTE_LAYERS), the other buses none;
3. a SHARED ARRAY: an escape bus on an array an earlier bus was handed on through gets that array planned whole
   (ESCAPE_JOINT) and its ends on the board as it was before that earlier bus (ESCAPE_JOINT_ENDS); an escape bus on
   arrays nothing went through, and one after an earlier bus that was not handed on, get neither; a plain or
   whole-array bus from such an array gets neither either, and no SOURCE_SHARED set on it;
3a. buses off one array the negotiation takes (an escape bus after the first: route_on_field runs it plain on the
   shared field; never first) are NEGOTIATED and that is their step -- none routed in turn before it, none alone,
   and what the negotiation leaves open is the step's result; --sequential negotiates none;
3b. keeps_source_escapes: a PLAIN bus keeps the escapes an earlier step laid its nets at its source by default, a
   whole-array or an escape bus does not; SOURCE_SHARED=0 keeps none; SOURCE_SHARED=REF,.. a plain or whole-array bus
   from those sources, never an escape bus;
4. step_policy lays a step's settings over the environment, the step's own policy over both.
"""
import json
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
check(rb.parse_bus('U1:U2:whole-array') == ('U1', 'U2', 'whole-array', None), '1: the whole-array mode')
for bad in ('U1', 'U1:U2:fast', ':U2', 'U1:U2:joint', 'U1:U2:whole-array:F.Cu:x'):
    try:
        rb.parse_bus(bad)
        check(False, f'1: {bad!r} refused')
    except ValueError:
        pass


# 2 and 3. route_buses with the step stood in for, and the negotiation (its groups kept in NEG)
import negotiated_buses as nb  # noqa: E402
NEG = []


SOLO, KEPT, ALONE = [], [], []


def run(buses, fail=(), sequential=False, left=None, neg_left=0):
    """route_buses with the step and the negotiation stood in for: `left` {(src, dest): [nets]} a step hands back,
    `neg_left` the nets the negotiation leaves open (its every bus, all told). SOLO the runs the negotiation was
    given teeth from, KEPT where each run of the summary came from ('negotiated' or 'in turn'), ALONE the buses
    routed alone"""
    calls = []
    del NEG[:], SOLO[:], KEPT[:], ALONE[:]

    def graded(open_):
        """a board's grade as route_bus.board_grade gives it: these bus nets open, no violation, no ball walled in"""
        return {'open': sorted(open_), 'drc': {'n': 0}, 'walled': {'walled': []}}

    def negotiate(board, out, grp, k, rounds, inproc, log, step_json, *a, solo=None, **kw):
        NEG.append(list(grp))
        SOLO.append([None if s_ is None else tuple(os.path.basename(p_) for p_ in s_) for s_ in solo or []])
        rows = []
        for i_, g_ in enumerate(grp):
            sp_ = step_json[:-len('.json')] + f'.{g_[1]}.json'
            json.dump({'routed': ['a'], 'refused': {f'n{j_}': 'open' for j_ in range(neg_left if i_ == 0 else 0)}},
                      open(sp_, 'w'))
            rows.append(dict(src=g_[0], dest=g_[1], mode=g_[2], exit=0, grade='', summary=sp_))
        # (...with the merged board's grade, as route_negotiated records it)
        json.dump({'buses': rows, 'grade': graded([f'n{j_}' for j_ in range(neg_left)])}, open(step_json, 'w'))
        shutil.copy(board, out)
        shutil.copy(board[:-len('.kicad_pcb')] + '.kicad_pro', out[:-len('.kicad_pcb')] + '.kicad_pro')
        return 0, []

    def step(board, out, src, dest, *a, whole_array=False, escape=False, settings=None, json_out=None, **kw):
        shutil.copy(board, out)
        shutil.copy(board[:-len('.kicad_pcb')] + '.kicad_pro', out[:-len('.kicad_pcb')] + '.kicad_pro')
        alone_ = bool(json_out) and json_out.endswith('.alone.summary.json')
        if alone_:
            ALONE.append((src, dest, os.path.basename(board)))
        else:
            calls.append(dict(board=board, src=src, dest=dest, whole_array=whole_array, escape=escape,
                              settings=dict(settings or {})))
        if json_out:
            json.dump({'routed': ['a'], 'refused': {n_: 'open' for n_ in (left or {}).get((src, dest), [])}},
                      open(json_out, 'w'))
        return (1 if (src, dest) in fail else 0), f'BUS {src}->{dest}'
    real, real_n = rb.route_bus, nb.route_negotiated
    rb.route_bus, nb.route_negotiated = step, negotiate
    try:
        with tempfile.TemporaryDirectory() as td:
            b0 = os.path.join(td, 'b.kicad_pcb')
            open(b0, 'w').write('(kicad_pcb)\n')
            open(os.path.join(td, 'b.kicad_pro'), 'w').write('{}\n')
            code, _g = rb.route_buses(b0, os.path.join(td, 'o.kicad_pcb'), buses, log=lambda *a: None,
                                      sequential=sequential)
            sj = os.path.join(td, 'o.bus', 'summary.json')
            KEPT.extend(('negotiated' if r_.get('negotiated') else 'in turn')
                        for r_ in (json.load(open(sj)).get('buses') or []) if os.path.isfile(sj))
            return code, calls, b0, os.path.join(td, 'o.bus')
    finally:
        rb.route_bus, nb.route_negotiated = real, real_n


L4 = 'F.Cu,B.Cu,In1.Cu,In2.Cu'
code, calls, b0, work = run([('U1', 'U2', 'whole-array', None), ('U1', 'U5', 'escape', L4),
                             ('U7', 'U8', 'escape', None)])
check(code == 0 and [(c['src'], c['dest']) for c in calls] == [('U1', 'U2'), ('U1', 'U5'), ('U7', 'U8')],
      f'2: the buses in the order given: {[(c["src"], c["dest"]) for c in calls]}')
check(calls[0]['whole_array'] and not calls[0]['escape'] and calls[0]['settings'] == {},
      f'2: U1-U2 whole-array: {calls[0]}')
check(calls[1]['escape'] and not calls[1]['whole_array'], f'2: U1-U5 escape: {calls[1]}')
check(calls[1]['board'] == os.path.join(work, 'U1-U2.kicad_pcb'), '2: U1-U5 on the board U1-U2 handed on')
check(calls[1]['settings'] == {'ROUTE_LAYERS': L4, 'ESCAPE_JOINT': 'U1', 'ESCAPE_JOINT_ENDS': b0},
      f'3: U1-U5: its layers, U1 shared, its ends on the board before U1-U2: {calls[1]["settings"]}')
check(calls[2]['escape'] and calls[2]['settings'] == {}, f'3: U7-U8: nothing shared: {calls[2]["settings"]}')
_c, calls, _b, _w = run([('U1', 'U2', 'whole-array', None), ('U1', 'U5', 'escape', None)], fail={('U1', 'U2')},
                        sequential=True)
check('ESCAPE_JOINT' not in calls[1]['settings'], f'3: after a bus left as it came, nothing shared: {calls[1]}')
_c, calls, _b, _w = run([('U1', 'U2', 'whole-array', None), ('U1', 'U5', 'whole-array', None)])
check(calls[1]['settings'] == {},
      f'3: a whole-array bus on a shared array: no ESCAPE_JOINT, no SOURCE_SHARED: {calls[1]}')
# (the zynq's four: the DDR3, the LVDS round it, then its USB and Ethernet buses to their QFNs)
_c, calls, _b, _w = run([('U1', 'U2', 'whole-array', None), ('U1', 'U5', 'escape', L4), ('U1', 'U4', 'plain', L4),
                         ('U1', 'U3', 'plain', L4), ('U7', 'U8', 'plain', None)])
check([c['settings'].get('SOURCE_SHARED') for c in calls] == [None] * 5,
      f'3: SOURCE_SHARED is the environment\'s to opt into, set on no bus: {[c["settings"] for c in calls]}')
check(calls[2]['settings'] == {'ROUTE_LAYERS': L4} and not calls[2]['escape'] and not calls[2]['whole_array'],
      f'3: U1-U4: its layers, plain: {calls[2]}')

# 3a. buses off one array the negotiation takes (an escape bus after the first: route_on_field runs it plain on the
# shared field) are NEGOTIATED, and that is their step: no bus of them routed in turn before it, none alone, no run's
# teeth handed to it
G3 = [('U1', 'U2', 'whole-array', L4), ('U1', 'U5', 'escape', L4), ('U1', 'U4', 'plain', L4)]
_c, calls, _b, _w = run(G3)
check(not calls and NEG == [G3] and not ALONE and KEPT == ['negotiated'] * 3,
      f'3a: the group negotiated, none of it in turn or alone: {NEG} {calls} {ALONE} {KEPT}')
check(SOLO == [[]], f'3a: no run before it to take teeth from: {SOLO}')
# ...and what the negotiation leaves open is the step's result: no pass in turn to fall back on
_c, calls, _b, _w = run(G3, neg_left=5)
check(not calls and NEG == [G3] and not ALONE and KEPT == ['negotiated'] * 3,
      f'3a: the negotiation leaving nets open: its board the step\'s all the same: {NEG} {calls} {ALONE} {KEPT}')
# ...a bus off another array after the group is routed in its turn, on the board the negotiation left
_c, calls, _b, w_ = run(G3 + [('U7', 'U8', 'plain', None)])
check(NEG == [G3] and [(c['src'], c['dest']) for c in calls] == [('U7', 'U8')]
      and calls[0]['board'] == os.path.join(w_, 'U1-U2_U5_U4.kicad_pcb'),
      f'3a: a bus off another array after the group, on the group\'s board: {NEG} {calls}')
check(nb.check([('U1', 'U5', 'escape', L4), ('U1', 'U2', 'plain', L4)]) is not None, '3a: an escape bus first is not')
_c, calls, _b, _w = run([('U1', 'U2', 'whole-array', L4), ('U1', 'U5', 'escape', L4)], sequential=True,
                        left={('U1', 'U5'): ['N1']})
check(not NEG and calls[1]['settings'].get('ESCAPE_JOINT') == 'U1', f'3a: --sequential: {NEG} {calls}')

# 3b. keeps_source_escapes (src, escape, whole_array, SOURCE_SHARED)
ks = rb.keeps_source_escapes
check(ks('U1', False, False, None) and ks('U1', False, False, '  '), '3b: a plain bus keeps them by default')
check(not ks('U1', False, True, None), '3b: a whole-array bus does not by default')
check(not ks('U1', True, False, None) and not ks('U1', True, False, 'U1'), '3b: an escape bus never')
check(not ks('U1', False, False, '0'), '3b: SOURCE_SHARED=0 keeps none')
check(ks('U1', False, True, 'U7,U1') and not ks('U1', False, False, 'U7'),
      '3b: SOURCE_SHARED=REF,..: a plain or whole-array bus from those sources only')

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
