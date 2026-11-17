#!/usr/bin/env python3
"""The bus step in the board's redo manifest (#132), its --find listing as JSON, and its events file.

  python3 tests/test_bus_step_record.py

route_bus.py records its own invocation, like every other board-writing CLI, so a chain's bus step is in
redo_commands.sh -- and only its own line: its stages (a cap pass, a check, each bus's own process) are its insides,
replayed by replaying the step. Checks:

1. --find --json-out FILE on a board with a bus: FILE holds the buses (src, dest, nets, why), the --bus arguments that
   name them and the board's ball arrays; with REDO_MANIFEST set, NO manifest line (--find routes nothing);
2. a run on a board with no bus, REDO_MANIFEST set: the manifest holds exactly the route_bus.py command, and the run's
   events file (<OUT stem>.bus/events.jsonl) says the step started, found nothing and ended with status 1;
3. a refused run (a part the board does not have) leaves no record (#1640);
4. a run that routes a bus (a 12-net synthetic pair, about 30 s): the manifest still holds only the step's own line --
   none of its stages', which self-record when they see a manifest -- and the events file has an event from each
   stage that ran: capture, the group's model, routing, fanout, solve, loop, lay, grade, the bus's process exit;
5. the --rounds / --feedback-rounds defaults are routing_defaults' (the GUI passes a flag only where its control
   differs from them).
"""
import json
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'tests'), os.path.join(ROOT, 'awx'), os.path.join(ROOT, 'py_router')]
from run_utils import check, evidence  # noqa: E402

TOOL = os.path.join(ROOT, 'awx', 'route_bus.py')
NO_BUS = os.path.join(ROOT, 'tests', 'fixtures', 'run25', 'esp_prog_placed.kicad_pcb')

failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + str(detail)) if detail and not cond else ''}")
    if not cond:
        failures.append(label)


def commands(manifest):
    if not os.path.isfile(manifest):
        return []
    return [ln for ln in open(manifest, encoding='utf-8').read().splitlines()
            if ln and not ln.startswith('#') and ln != 'set -e']


def main():
    td = tempfile.mkdtemp(prefix='bus_record_')
    board = os.path.join(td, 'syn.kicad_pcb')
    check([sys.executable, os.path.join(ROOT, 'awx', 'synth_bus.py'), board, '--pattern', 'sorted', '--k', '12',
           '--cols', '7', '--depth', '4', '--copper', '4'], accept=True, timeout=120)
    evidence(board, 'the synthetic board')

    print('1. --find --json-out: the listing as JSON, and no manifest line')
    man = os.path.join(td, 'find_redo.sh')
    found = os.path.join(td, 'found.json')
    env = dict(os.environ, REDO_MANIFEST=man, REDO_RUNDIR=td)
    r = check([sys.executable, '-X', 'utf8', TOOL, board, '--find', '--json-out', found, '--clearance-ceiling', '0.1',
               '--track-width', '0.1'], accept=True, timeout=300, env=env, cwd=td)
    j = json.load(open(evidence(found, '--find listing'), encoding='utf-8'))
    ok('one bus, SD1 -> SU1, 12 nets', [(b['src'], b['dest'], b['nets']) for b in j.get('buses', [])]
       == [('SD1', 'SU1', 12)], j.get('buses'))
    ok('...with why', bool(j['buses'] and j['buses'][0].get('why')))
    ok('the --bus arguments', j.get('step') == ['SD1:SU1'], j.get('step'))
    ok('the board\'s ball arrays', j.get('arrays') == ['SD1', 'SU1'], j.get('arrays'))
    ok('left: a list', isinstance(j.get('left'), list))
    ok('the printed listing too', 'the step: --bus SD1:SU1' in r.stdout, r.stdout[-300:])
    ok('no manifest line for --find', commands(man) == [], commands(man))

    print('2. a run with REDO_MANIFEST: its own line, and its events')
    man = os.path.join(td, 'redo_commands.sh')
    # (KICAD_FORCE_COLOR: the plugin sets it in KiCad's process, so the Bus tab's child inherits it -- a console
    # colour, no knob a replay needs, so the line carries no prefix for it)
    env = dict(os.environ, REDO_MANIFEST=man, REDO_RUNDIR=td, KICAD_FORCE_COLOR='1')
    env.pop('BUS_EVENTS', None)
    out = os.path.join(td, 'none.kicad_pcb')
    check([sys.executable, '-X', 'utf8', TOOL, NO_BUS, out], accept=True, timeout=600, env=env, cwd=td)
    cmds = commands(man)
    ok('exactly one command recorded', len(cmds) == 1, cmds)
    ok('...route_bus.py with its board and OUT, no env prefix (KICAD_FORCE_COLOR is no replay knob)',
       bool(cmds) and 'route_bus.py' in cmds[0] and 'none.kicad_pcb' in cmds[0]
       and cmds[0].startswith('python3 -u -X utf8'), cmds)
    evs = [json.loads(ln) for ln in open(evidence(os.path.join(td, 'none.bus', 'events.jsonl'), 'events'),
                                         encoding='utf-8')]
    ok('events: step start, plan, end', [(e['stage'], e['kind']) for e in evs]
       == [('step', 'start'), ('step', 'plan'), ('step', 'end')], [(e['stage'], e['kind']) for e in evs])
    ok('...the plan found no bus, the step ended with status 1',
       evs[1].get('buses') == [] and evs[-1].get('status') == 1, evs)

    print('3. a refused run leaves no record')
    man3 = os.path.join(td, 'refused_redo.sh')
    env = dict(os.environ, REDO_MANIFEST=man3, REDO_RUNDIR=td)
    check([sys.executable, '-X', 'utf8', TOOL, NO_BUS, os.path.join(td, 'bad.kicad_pcb'), '--bus', 'U1:NOPE9'],
          refuse='no footprint NOPE9', code=2, timeout=300, env=env, cwd=td)
    ok('no manifest line', commands(man3) == [], commands(man3))

    print('4. a run that routes a bus: still only its own line -- none of its stages\' (about 30 s)')
    man5 = os.path.join(td, 'routed_redo.sh')
    env = dict(os.environ, REDO_MANIFEST=man5, REDO_RUNDIR=td)
    env.pop('BUS_EVENTS', None)
    out5 = os.path.join(td, 'bus.kicad_pcb')
    check([sys.executable, '-X', 'utf8', TOOL, board, out5, '--rounds', '1', '--clearance-ceiling', '0.1',
           '--track-width', '0.1', '--via-size', '0.35', '--via-drill', '0.15'], accept=True, timeout=900, env=env,
          cwd=td)
    s5 = json.load(open(evidence(os.path.join(td, 'bus.bus', 'summary.json'), 'summary'), encoding='utf-8'))
    ok('the bus routed (the step\'s status 0)', s5.get('exit') == 0, s5.get('exit'))
    cmds = commands(man5)
    ok('exactly one command recorded, route_bus.py', len(cmds) == 1 and 'route_bus.py' in cmds[0], cmds)
    kinds = [(e['stage'], e['kind']) for e in (json.loads(ln) for ln in open(
        evidence(os.path.join(td, 'bus.bus', 'events.jsonl'), 'events'), encoding='utf-8'))]
    for want in (('capture', 'end'), ('ends', 'solve'), ('route', 'start'), ('fanout', 'end'), ('solve', 'end'),
                 ('loop', 'end'), ('lay', 'end'), ('bus', 'graded'), ('bus', 'exit'), ('step', 'end')):
        ok(f'an event {want[0]} {want[1]}, from the stage that ran it', want in kinds)

    print('5. the rounds defaults are routing_defaults\'')
    import routing_defaults as d
    r = check([sys.executable, '-X', 'utf8', TOOL, '--help'], accept=True, timeout=120)
    ok(f'--rounds default {d.BUS_ROUNDS} in --help', f'(default {d.BUS_ROUNDS})' in ' '.join(r.stdout.split()))
    ok('BUS_FEEDBACK_ROUNDS is 0 (every round a draw)', d.BUS_FEEDBACK_ROUNDS == 0)

    print('\nFAILED: ' + ', '.join(failures) if failures else '\nALL OK')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
