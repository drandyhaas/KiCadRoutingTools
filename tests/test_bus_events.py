#!/usr/bin/env python3
"""The bus step's progress events (awx/bus_events.py), the file the GUI's Bus tab follows.

  python3 tests/test_bus_events.py

1. with no BUS_EVENTS setting, emit writes nothing anywhere;
2. run(WORK) names WORK/events.jsonl for the block and starts it empty; a caller's BUS_EVENTS is appended to, never
   emptied, and stays the file inside;
3. a process routing one bus (for_bus) has its events carry that bus and its source as the group; an event naming
   no bus (bus=None) keeps the group; one that names a group and a bus carries those; phase "capture" under
   ENDS_ONLY, for a bus's events only;
4. line-atomic: eight processes writing 200 events each at once leave 1600 whole lines, every one JSON;
5. read(): whole lines only -- a line still being written is left for the next read, which resumes at the offset;
6. a write that cannot be made (the file's folder gone) is dropped, never raised.
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'awx')]
import awx_settings  # noqa: E402
import bus_events  # noqa: E402

failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + str(detail)) if detail and not cond else ''}")
    if not cond:
        failures.append(label)


def lines(path):
    with open(path, encoding='utf-8') as f:
        return [json.loads(ln) for ln in f if ln.strip()]


def main():
    td = tempfile.mkdtemp(prefix='bus_events_')
    env0 = {k: v for k, v in os.environ.items() if k not in (bus_events.ENV, bus_events.BUS, 'ENDS_ONLY')}

    print('1. no BUS_EVENTS: nothing written')
    with awx_settings.given(env0):
        before = set(os.listdir(td))
        bus_events.emit('step', 'start', board='x')
        ok('no file appears', set(os.listdir(td)) == before)

    print('2. run(WORK): the step\'s file, started empty; a given one appended to')
    work = os.path.join(td, 'out.bus')
    os.makedirs(work)
    stale = os.path.join(work, bus_events.NAME)
    with open(stale, 'w') as f:
        f.write('{"stale": true}\n')
    with awx_settings.given(env0):
        with bus_events.run(work) as path:
            ok('the path is WORK/events.jsonl', path == stale, path)
            ok('...started empty', not os.path.exists(path) or os.path.getsize(path) == 0)
            ok('...and given to everything inside', awx_settings.get(bus_events.ENV) == path)
            bus_events.emit('step', 'start', n=1)
            with bus_events.run(os.path.join(td, 'other.bus')) as inner:
                ok('a nested run keeps the given file', inner == path, inner)
                bus_events.emit('step', 'end', status=0)
        ok('outside the block the setting is gone', not awx_settings.get(bus_events.ENV))
    got = lines(stale)
    ok('two events, in order', [e['kind'] for e in got] == ['start', 'end'], got)
    ok('each with v, t, pid, stage, kind, group, bus', all(
        {'v', 't', 'pid', 'stage', 'kind', 'group', 'bus'} <= set(e) for e in got))
    ok('the payload kept', got[0].get('n') == 1 and got[1].get('status') == 0)
    ok('the nested run wrote no file of its own', not os.path.exists(os.path.join(td, 'other.bus')))

    print('3. a bus\'s process: its events carry its bus and its source')
    path = os.path.join(td, 'ctx.jsonl')
    with awx_settings.given({**env0, bus_events.ENV: path, **bus_events.for_bus('U1', 'U4+U3')}):
        bus_events.emit('fanout', 'start', round=1)
        bus_events.emit('coupled', 'search', bus=None, round=1)
        bus_events.emit('capture', 'end', group='U9', bus='U9:J2')
    with awx_settings.given({**env0, bus_events.ENV: path, **bus_events.for_bus('U1', 'U2'), 'ENDS_ONLY': '1'}):
        bus_events.emit('solve', 'end', status='OPTIMAL')
        bus_events.emit('ends', 'solve', group='U1', bus=None)
    a, b, c, d, e = lines(path)
    ok('own bus and group', (a['bus'], a['group']) == ('U1:U4+U3', 'U1'), a)
    ok('bus=None keeps the process\'s group', (b['bus'], b['group']) == (None, 'U1'), b)
    ok('named group and bus win', (c['bus'], c['group']) == ('U9:J2', 'U9'), c)
    ok('a capture run\'s bus event says phase capture', d.get('phase') == 'capture', d)
    ok('...a group event in it does not', 'phase' not in e, e)

    print('4. line-atomic appends from eight processes at once')
    path = os.path.join(td, 'many.jsonl')
    code = ('import sys; sys.path.insert(0, sys.argv[1]); import bus_events\n'
            'for i in range(200): bus_events.emit("solve", "end", n=i, pad="x" * 300)')
    env = {**env0, bus_events.ENV: path}
    ps = [subprocess.Popen([sys.executable, '-c', code, os.path.join(ROOT, 'awx')], env=env) for _ in range(8)]
    rcs = [p.wait(timeout=120) for p in ps]
    ok('every writer exited 0', rcs == [0] * 8, rcs)
    raw = open(path, encoding='utf-8').read().splitlines()
    parsed = []
    for ln in raw:
        try:
            parsed.append(json.loads(ln))
        except ValueError:
            pass
    ok('1600 lines, every one whole JSON', len(raw) == 1600 and len(parsed) == 1600, (len(raw), len(parsed)))
    ok('...200 from each process', sorted({p_['pid'] for p_ in parsed}).__len__() == 8 and all(
        sum(1 for p_ in parsed if p_['pid'] == pid) == 200 for pid in {p_['pid'] for p_ in parsed}))

    print('5. read(): whole lines only, resumed at the offset')
    path = os.path.join(td, 'partial.jsonl')
    with open(path, 'wb') as f:
        f.write(b'{"stage": "a", "kind": "start"}\nnot json\n{"stage": "b", "kind": "st')
    evs, off = bus_events.read(path)
    ok('the whole lines read, the bad one skipped', [e['stage'] for e in evs] == ['a'], evs)
    with open(path, 'ab') as f:
        f.write(b'art"}\n')
    evs2, off2 = bus_events.read(path, off)
    ok('the finished line read next time', [e['stage'] for e in evs2] == ['b'] and off2 == os.path.getsize(path),
       (evs2, off2))
    ok('a missing file reads as nothing', bus_events.read(os.path.join(td, 'nope.jsonl'), 7) == ([], 7))
    ok('default_path is <OUT stem>.bus/events.jsonl',
       bus_events.default_path('/x/y/bus.kicad_pcb') == os.path.join('/x/y/bus.bus', bus_events.NAME))

    print('6. a write that cannot be made is dropped')
    with awx_settings.given({**env0, bus_events.ENV: os.path.join(td, 'gone', 'deeper', 'e.jsonl')}):
        try:
            bus_events.emit('step', 'start')
            ok('no exception', True)
        except Exception as ex:                       # noqa: BLE001
            ok('no exception', False, repr(ex))

    print('\nFAILED: ' + ', '.join(failures) if failures else '\nALL OK')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
