#!/usr/bin/env python3
"""The Bus tab's wx-free half (kicad_routing_plugin/bus_progress.py): the command it runs, the buses it names, and its
model of the step's events.

  python3 tests/test_bus_gui_model.py

1. build_argv is the CLI's command: the defaults give the bare `route_bus.py BOARD OUT`, a flag only where a control
   differs from route_bus's own default (routing_defaults), --bus in the routing order, sizes in route_bus.SIZE_ARGS
   order -- the order read from route_bus.py itself; find_argv never passes the escapes' track; a bad spec refused;
2. BusList: --find's buses, all checked, name none (the step finds them itself); an unchecked, moved, added or edited
   one names every checked row with --bus, in order; back to --find's names none again; a plan's list names its own;
3. RunModel on a negotiated group of two buses (events as the step writes them): the rows in order, every bus's
   result, its stages, a capture run's stages kept under its capture row, a loop's re-solve under the loop, the
   coupled solve under the routing stage, and the gauges' fractions -- the group's stages ended of those that will
   run, its buses finished of the routing stage's;
4. a bus whose process died (no end event) is FAILED with its log; the step's end closes what is still open;
5. describe() says something for every kind the step writes;
6. the python probe's verdict: KiCad's own python (pcbnew importable) is never offered an OR-Tools install -- OR-Tools
   crashed beside pcbnew there --, a python lacking only OR-Tools is; a probe that died loading pcbnew names the clash.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [ROOT, os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'awx')]
from kicad_routing_plugin import bus_progress as bp  # noqa: E402
import routing_defaults as defaults  # noqa: E402

failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + str(detail)) if detail and not cond else ''}")
    if not cond:
        failures.append(label)


def ev(t, stage, kind, group=None, bus=None, **kw):
    return dict(v=1, t=1000.0 + t, pid=1, stage=stage, kind=kind, group=group, bus=bus, **kw)


def group_events():
    """A two-bus group U1 -> U2, U1 -> J4+J5, as the step writes it (negotiated, one round)."""
    a, b = 'U1:U2', 'U1:J4+J5'
    return [
        ev(0, 'step', 'start', buses=None, rounds=1),
        ev(1, 'step', 'plan', buses=[dict(src='U1', dest='U2', nets=22, why='between two ball arrays'),
                                     dict(src='U1', dest='J4+J5', nets=20, why='row parts')],
           left=[dict(src='U3', dest='J9', nets=5, why='a track passes between its pads')]),
        ev(1, 'group', 'start', group='U1', buses=[a, b], work='/w/U1-U2_J4+J5.bus'),
        ev(1, 'capture', 'start', group='U1', buses=[a, b]),
        ev(3, 'fanout', 'start', group='U1', bus=a, phase='capture', round=1),
        ev(9, 'capture', 'end', group='U1', bus=b, ok=True, secs=8, log='/w/cap/J4+J5.log'),
        ev(12, 'capture', 'end', group='U1', bus=a, ok=True, secs=11, log='/w/cap/U2.log'),
        ev(12, 'ends', 'start', group='U1', round=1, rounds=1, log='/w/group.log'),
        ev(20, 'ends', 'solve', group='U1', check=1, status='OPTIMAL', secs=8, rest=12.1, bound=None, walled=0,
           changing_layer=3),
        ev(25, 'ends', 'check', group='U1', check=1, refused=['J4+J5']),
        ev(40, 'ends', 'solve', group='U1', check=2, status='OPTIMAL', secs=15, rest=11.0, bound=None, walled=0,
           changing_layer=2),
        ev(41, 'ends', 'field', group='U1', drc=0, drc_bare=0, bad=False, tracks=40, vias=4),
        ev(41, 'ends', 'done', group='U1', secs=29, refused=[], no_plan=[]),
        ev(42, 'ends', 'end', group='U1', round=1, exit=0, secs=30, log='/w/group.log'),
        ev(42, 'route', 'start', group='U1', buses=[a, b], negotiated=True),
        ev(43, 'bus', 'start', group='U1', bus=a, nets=22, k=22),
        ev(43, 'bus', 'start', group='U1', bus=b, nets=20, k=20),
        ev(44, 'fanout', 'start', group='U1', bus=a, round=1, log='/w/a/fo.log'),
        ev(45, 'fanout', 'end', group='U1', bus=a, round=1, exit=0, secs=2),
        ev(45, 'solve', 'start', group='U1', bus=a, round=1),
        ev(46, 'solve', 'end', group='U1', bus=a, status='FEASIBLE', secs=1, crossings=6, proved=False, plan=True,
           obj=9, bound=9),
        ev(46, 'coupled', 'start', group='U1', bus=a, round=1),
        ev(60, 'coupled', 'search', group='U1', bus=None, round=1, status='FEASIBLE', secs=14, cost=25.5,
           bound=25.0, broken=3, couplings=200),
        ev(61, 'coupled', 'end', group='U1', bus=a, round=1, joint=True, own=False, secs=15),
        ev(61, 'loop', 'start', group='U1', bus=a, round=1, log='/w/a/loop.log'),
        ev(70, 'solve', 'end', group='U1', bus=a, status='OPTIMAL', secs=3, crossings=6, proved=True, plan=True,
           obj=8, bound=8),
        ev(90, 'loop', 'end', group='U1', bus=a, round=1, exit=0, secs=46),
        ev(90, 'lay', 'start', group='U1', bus=a, round=1),
        ev(95, 'lay', 'end', group='U1', bus=a, round=1, laid=True, lanes='22/22', open=0, drc=0, vias=30,
           copper_mm=400),
        ev(96, 'bus', 'graded', group='U1', bus=a, vias=30, copper_mm=401, connected=True, drc=0, walled=0),
        ev(96, 'bus', 'end', group='U1', bus=a, exit=0, routed=22, refused=0, secs=53),
        ev(97, 'bus', 'exit', group='U1', bus=a, exit=0, secs=55, log='/w/neg/U2/bus.log'),
        ev(98, 'bus', 'exit', group='U1', bus=b, exit=-9, secs=56, log='/w/neg/J4+J5/bus.log'),
        ev(99, 'merge', 'end', group='U1', conflicts=0, given_back=[]),
        ev(100, 'grade', 'end', group='U1', open=20, drc=0, walled=0),
        ev(100, 'group', 'end', group='U1', exit=1, open=20, drc=0, walled=0),
        ev(101, 'step', 'end', status=1, out='/w/bus.kicad_pcb'),
    ]


def main():
    print('1. build_argv: the CLI\'s own command')
    base = ['py', '-u', '-X', 'utf8', bp.ROUTE_BUS_SCRIPT, 'in.kicad_pcb', 'out.kicad_pcb']
    ok('defaults: the bare command', bp.build_argv('py', 'in.kicad_pcb', 'out.kicad_pcb', None, {
        'rounds': defaults.BUS_ROUNDS, 'feedback_rounds': defaults.BUS_FEEDBACK_ROUNDS, 'k': 0,
        'inproc': False, 'round_edits': ''}, {}) == base)
    argv = bp.build_argv('py', 'in.kicad_pcb', 'out.kicad_pcb', ['U1:U2', 'U1:J4+J5'],
                         {'rounds': 1, 'feedback_rounds': 2, 'k': 12, 'inproc': True, 'round_edits': '/e.json'},
                         {'via_size': 0.35, 'clearance_ceiling': 0.1, 'track_width': 0.1, 'fanout_track_width': 0.08,
                          'clearance': None})
    ok('every option, the buses in order, the sizes in SIZE_ARGS order', argv == base + [
        '--bus', 'U1:U2', '--bus', 'U1:J4+J5', '--rounds', '1', '--feedback-rounds', '2', '--k', '12', '--inproc',
        '--round-edits', '/e.json', '--clearance-ceiling', '0.1', '--track-width', '0.1',
        '--fanout-track-width', '0.08', '--via-size', '0.35'], argv)
    ok('SIZE_ARGS read from route_bus.py', bp.SIZE_ARGS[:2] == ('clearance', 'clearance_ceiling')
       and 'fanout_track_width' in bp.SIZE_ARGS and len(bp.SIZE_ARGS) == 8, bp.SIZE_ARGS)
    import route_bus
    ok('...exactly route_bus.SIZE_ARGS', bp.SIZE_ARGS == tuple(route_bus.SIZE_ARGS))
    fa = bp.find_argv('py', 'in.kicad_pcb', '/f.json', {'track_width': 0.1, 'fanout_track_width': 0.08})
    ok('find_argv: --find, its listing, the sizes but the escapes\' track', fa == [
        'py', '-u', '-X', 'utf8', bp.ROUTE_BUS_SCRIPT, 'in.kicad_pcb', '--find', '--json-out', '/f.json',
        '--track-width', '0.1'], fa)
    for bad in ('U1', 'U1:', ':U2', 'U1:U2:U3', 'U1:U2+'):
        try:
            bp.build_argv('py', 'a', 'b', [bad])
            ok(f'{bad!r} refused', False)
        except ValueError:
            ok(f'{bad!r} refused', True)

    print('2. BusList')
    bl = bp.BusList()
    bl.set_found({'buses': [dict(src='U1', dest='U2', nets=22, why='w'), dict(src='U1', dest='J4', nets=10, why='x')],
                  'left': [dict(src='U3', dest='J9', nets=5, why='y')], 'arrays': ['U1', 'U2']})
    ok("--find's buses: the run names none", bl.argv_buses() is None)
    bl.rows[1]['checked'] = False
    ok('one unchecked: the rest named', bl.argv_buses() == ['U1:U2'], bl.argv_buses())
    bl.rows[1]['checked'] = True
    bl.move(1, -1)
    ok('moved: named in the new order', bl.argv_buses() == ['U1:J4', 'U1:U2'], bl.argv_buses())
    bl.use_found()
    ok("back to --find's: none named", bl.argv_buses() is None and bl.checked() == ['U1:U2', 'U1:J4'])
    bl.add('U1:J5+J6')
    ok('added: every checked row named', bl.argv_buses() == ['U1:U2', 'U1:J4', 'U1:J5+J6'])
    bl.replace(2, 'U1:J6')
    ok('edited', bl.argv_buses() == ['U1:U2', 'U1:J4', 'U1:J6'])
    bl.set_named(['U1:J4'])
    ok("a plan's list: its own, its rows keep --find's nets", bl.argv_buses() == ['U1:J4'] and bl.rows[0]['nets'] == 10)
    empty = bp.BusList()
    ok('no --find yet and no rows: none named', empty.argv_buses() is None)
    empty.set_named(['U1:U2'])
    ok('no --find yet, a plan\'s list: named', empty.argv_buses() == ['U1:U2'])

    print('3. RunModel: a negotiated group of two buses')
    m = bp.RunModel()
    m.feed_all(group_events())
    flat = m.flat()
    labels = [(r['level'], r['label']) for r in flat]
    ok('the group first, then its stages, then each bus with its stages',
       labels[0] == (0, 'U1 group') and [l for l, _ in labels[:6]] == [0, 2, 2, 2, 2, 2], labels[:8])
    g = flat[0]
    gst = [r for r in flat if r['level'] == 2 and r['parent'] == g['key']]
    ok("the group's stages: capture, ends, route, merge, grade",
       [r['stage'] for r in gst] == ['capture', 'ends', 'route', 'merge', 'grade'], [r['stage'] for r in gst])
    ok('...all ended', all(r['state'] == 'done' for r in gst), [(r['stage'], r['state']) for r in gst])
    ends = gst[1]
    ok('the ends model: its solves and its check as details, its last as the result',
       len(ends['details']) == 5 and 'check 1' in ends['details'][1] and 'refused' in ends['details'][1]
       and ends['result'].startswith('the ends model ended'), (ends['details'], ends['result']))
    route = gst[2]
    ok('the coupled solve under the routing stage', any('coupled solve round 1: [FEASIBLE]' in d
                                                        for d in route['details']), route['details'])
    ok('the routing stage: 2 of 2 buses finished', route['result'] == '2 of 2 buses finished', route['result'])
    a = m.rows[('b', 'U1:U2')]
    ok('bus U1:U2 done, its result lanes, vias, copper, open, DRC, walled',
       a['state'] == 'done' and a['result'] == 'lanes 22/22; vias 30, copper 401 mm, open 0, DRC 0, walled 0',
       (a['state'], a['result']))
    ast_ = [r for r in flat if r['parent'] == a['key']]
    ok("its stages: capture, fanout, solve, coupled, loop, lay",
       [r['stage'] for r in ast_] == ['capture', 'fanout', 'solve', 'coupled', 'loop', 'lay'],
       [r['stage'] for r in ast_])
    ok('the capture run\'s fanout kept under its capture row',
       any('fanout r1' in d for d in ast_[0]['details']), ast_[0]['details'])
    ok("the loop's re-solve under the loop", any(d.startswith('re-solve: solve: [OPTIMAL]')
                                                  for d in ast_[4]['details']), ast_[4]['details'])
    ok('elapsed: the loop 29 s', abs(m.elapsed(ast_[4], 2000.0) - 29.0) < 1e-6, m.elapsed(ast_[4], 2000.0))

    print('4. a bus whose process died: FAILED with its log; the step end closes the rest')
    b = m.rows[('b', 'U1:J4+J5')]
    ok('FAILED, its log named', b['state'] == 'failed' and b['log'] == '/w/neg/J4+J5/bus.log', (b['state'], b['log']))
    ok('no stage left running', not any(r['state'] == 'running' for r in flat))
    ok('the step done, status 1', m.step == 'done' and m.status == 1 and m.out == '/w/bus.kicad_pcb')
    fr = m.fractions()
    ok('gauges: 5 of 5 group stages, 2 of 2 buses', fr == {'stages': (5, 5), 'buses': (2, 2)}, fr)
    part = bp.RunModel()
    part.feed_all(group_events()[:16])
    fr = part.fractions()
    ok('mid-run: 2 of 5 stages ended (capture, ends), 0 of 2 buses', fr == {'stages': (2, 5), 'buses': (0, 2)}, fr)
    ok('the status line names the latest event',
       part.status_line(1050.0).startswith('[0:43] U1:U2: 22 nets'), part.status_line(1050.0))
    two = bp.RunModel()
    two.feed_all(group_events()[:4] + [ev(12, 'ends', 'start', group='U1', round=2, rounds=2)])
    ok('a second group round adds its four stages to the count', two.fractions()['stages'] == (0, 9),
       two.fractions())

    print('5. describe() for every kind')
    for e in group_events():
        d = bp.describe(e)
        ok(f"{e['stage']} {e['kind']}: {d[:70]}", bool(d) and d != f"{e['stage']} {e['kind']}")

    print('6. the python probe\'s verdict')
    v = bp.probe_verdict(3, 'python 3.9.13\nPCBNEW_IMPORTABLE\nPROBLEM: route_bus.py does not parse\n'
                            'PROBLEM: ERROR: ... ortools: not installed\nNOT OK\n')
    ok('KiCad\'s python: both problems, pcbnew, NOT installable (OR-Tools beside pcbnew is no fix)',
       not v['ok'] and len(v['problems']) == 2 and v['pcbnew'] and not v['installable'], v)
    v = bp.probe_verdict(3, 'python 3.12.1\nPROBLEM: ERROR: ... ortools: not installed\nNOT OK\n')
    ok('a python without pcbnew lacking only OR-Tools: installable', v['installable'] and not v['ok'], v)
    v = bp.probe_verdict(-11, 'python 3.9.13\nPCBNEW_IMPORTABLE\nPCBNEW_CHECK\n')
    ok('a probe that died loading pcbnew beside OR-Tools: the clash named', not v['ok'] and any(
        bp.PCBNEW_CLASH in p_ for p_ in v['problems']) and not v['installable'], v)
    v = bp.probe_verdict(0, 'python 3.14.8\nOK\n')
    ok('a ready python', v['ok'] and not v['problems'], v)
    v = bp.probe_verdict(1, '')
    ok('no output at all: not ready, said so', not v['ok'] and v['problems'], v)

    print('\nFAILED: ' + ', '.join(failures) if failures else '\nALL OK')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
