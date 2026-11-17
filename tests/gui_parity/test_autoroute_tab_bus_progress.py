#!/usr/bin/env python3
"""The Autorouter tab's display of autoroute's bus step, on the REAL tab (headless, never shown), fed what autoroute
writes -- its AUTOROUTE_PLAN and AUTOROUTE_STEP lines -- and a bus step's events file written in pieces while the
step's row runs, the way route_bus writes it. Nothing routes, so it needs no OR-Tools.

  python3 tests/gui_parity/test_autoroute_tab_bus_progress.py

1. the Plan pane has the bus line: the bus, the arrays it serves, its via, the buses not taken;
2. the step's `running` event, naming its events file, makes the row follow it: after a poll the Result column (the
   list's own cell) shows the live line -- the latest event and the group-stage gauge --, and the row's details list
   the step's own rows under "the bus step's progress";
3. lines written later reach the next poll (and not before the tab's once-a-second wait);
4. the step's `done` event ends it: the last lines read, nothing of the step left running, the Result column back to
   the step's result;
5. the plan of a board whose bus step cannot run (no OR-Tools) says NOT RUN with the install line.

Needs KiCad python (wx); re-execs into it like its siblings, and exits 77 with a SKIP: reason when there is none.
"""
import glob
import json
import os
import subprocess
import sys
import tempfile
import time

os.environ.setdefault('WXSUPPRESS_SIZER_FLAGS_CHECK', '1')

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from kicad_locate import path_version_key  # noqa: E402
del sys.path[0]
KICAD_PYTHONS = [
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    "/usr/bin/python3",
    *sorted(glob.glob(r"C:\Program Files\KiCad\*\bin\python.exe"), key=path_version_key, reverse=True),
]
FAILS = []


def check(name, ok, detail=''):
    print(f"  {'PASS' if ok else 'FAIL'}: {name}" + (f" -- {detail}" if detail and not ok else ''), flush=True)
    if not ok:
        FAILS.append(name)


def _reexec():
    for cand in KICAD_PYTHONS:
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import wx'], capture_output=True).returncode == 0:
            sys.exit(subprocess.run([cand, os.path.abspath(__file__)] + sys.argv[1:]).returncode)
    print("SKIP: no python with wx found (this gate needs KiCad's)")
    sys.exit(77)


def ev(t, stage, kind, group=None, bus=None, **kw):
    return dict(v=1, t=t, pid=1, stage=stage, kind=kind, group=group, bus=bus, **kw)


def step_events(t0):
    a = 'U1:U2'
    return [
        ev(t0, 'step', 'start', buses=None, rounds=2),
        ev(t0 + 1, 'group', 'start', group='U1', buses=[a], work='/w/U1-U2.bus'),
        ev(t0 + 1, 'capture', 'start', group='U1', buses=[a]),
        ev(t0 + 5, 'capture', 'end', group='U1', bus=a, ok=True, secs=4, log='/w/cap.log'),
        ev(t0 + 5, 'ends', 'start', group='U1', round=1, rounds=1, log='/w/group.log'),
        ev(t0 + 9, 'ends', 'solve', group='U1', check=1, status='OPTIMAL', secs=4, rest=10.0, bound=None, walled=0,
           changing_layer=2),
        # (written while the row runs)
        ev(t0 + 12, 'ends', 'end', group='U1', round=1, exit=0, secs=7, log='/w/group.log'),
        ev(t0 + 12, 'route', 'start', group='U1', buses=[a], negotiated=False),
        ev(t0 + 13, 'bus', 'start', group='U1', bus=a, nets=34, k=34),
        ev(t0 + 30, 'lay', 'end', group='U1', bus=a, round=1, laid=True, lanes='34/34', open=0, drc=0, vias=34,
           copper_mm=503),
        ev(t0 + 31, 'bus', 'graded', group='U1', bus=a, vias=34, copper_mm=503, connected=True, drc=0, walled=0),
        ev(t0 + 31, 'bus', 'end', group='U1', bus=a, exit=0, routed=34, refused=0, secs=18),
        ev(t0 + 32, 'bus', 'exit', group='U1', bus=a, exit=0, secs=19, log='/w/neg/U2/bus.log'),
        ev(t0 + 33, 'grade', 'end', group='U1', open=0, drc=0, walled=0),
        ev(t0 + 33, 'group', 'end', group='U1', exit=0, open=0, drc=0, walled=0),
        ev(t0 + 34, 'step', 'end', status=0, out='/w/bus.kicad_pcb'),
    ]


def main():
    try:
        import wx
    except ImportError:
        _reexec()
        return
    sys.path.insert(0, REPO)
    from kicad_routing_plugin.autoroute_gui import AutorouteTab
    app = wx.App(False)  # noqa: F841
    frame = wx.Frame(None)
    tab = AutorouteTab(frame, None)
    tmp = tempfile.mkdtemp(prefix='krt_ar_bus_')
    events = os.path.join(tmp, 'step01_bus.bus', 'events.jsonl')
    os.makedirs(os.path.dirname(events))
    bus = {'buses': [{'src': 'U1', 'dest': 'U2', 'nets': 34, 'why': 'a memory bus'}], 'arrays': ['U1', 'U2'],
           'not_taken': [{'src': 'U1', 'dest': 'J1', 'nets': 16, 'why': 'an edge connector, both faces'}],
           'via': 0.45, 'drill': 0.2, 'missing': None, 'error': None}
    steps = [{'tag': 'bus', 'tool': 'bus', 'what': 'route 1 bus(es) whole with route_bus: U1 -> U2 (34 nets)',
              'why': 'EXPERIMENTAL', 'optional': False},
             {'tag': 'route', 'tool': 'route', 'what': 'route every net', 'why': 'grid 0.1 mm', 'optional': False}]
    plan = {'mode': 'full', 'n_nets': 120, 'layers': ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'], 'rules': {},
            'planes': [], 'parts': [], 'bus': bus, 'steps': steps}
    line = lambda tok, p: f"{tok}: {json.dumps(p)}"            # noqa: E731

    print('1. the plan')
    tab._handle_line(line('AUTOROUTE_PLAN', plan))
    text = tab.plan_text.GetValue()
    check("the Plan pane's bus line: the bus, the arrays, the via, the bus not taken",
          'Buses (the bus step, experimental): U1 -> U2 (34 nets)' in text and 'U1, U2' in text
          and '0.45/0.2' in text and 'U1 -> J1 (16 nets: an edge connector' in text, text)

    print('2. the step runs')
    t0 = time.time() - 40
    evs = step_events(t0)
    with open(events, 'w') as f:
        f.write(''.join(json.dumps(e) + '\n' for e in evs[:6]))
    run_ev = {'n': 1, 'tag': 'bus', 'tool': 'bus', 'parent': None, 'cmd': ['route_bus.py'],
              'log': os.path.join(tmp, 'step01_bus.log'), 'board': os.path.join(tmp, 'step01_bus.kicad_pcb'),
              'input': os.path.join(tmp, 'in.kicad_pcb'), 'events': events, 'state': 'running', 't': t0}
    tab._handle_line(line('AUTOROUTE_STEP', run_ev))
    row = tab.rows[0]
    check("the running row follows the named events file", row.get('tail') is not None
          and row['tail'].path == events)
    tab._tail_t = 0.0
    changed = tab._poll_tails()
    tab._refresh_list()
    live = row.get('live') or ''
    cell = tab.step_list.GetItemText(0, 4)
    check("a poll reads it: the live line, the latest event and the group-stage gauge",
          changed and 'U1' in live and 'group stages' in live, live)
    check("the Result column (the list's cell) shows the live line", cell == live, f"cell {cell!r}")
    tab._show_row(0)
    check("its details list the step's own rows", "the bus step's progress" in tab._detail_value
          and 'U1' in tab._detail_value, tab._detail_value[-500:])

    print('3. lines written later')
    with open(events, 'a') as f:
        f.write(''.join(json.dumps(e) + '\n' for e in evs[6:12]))
    check("not before the once-a-second wait", tab._poll_tails() is False)
    tab._tail_t = 0.0
    check("the next poll reads them", tab._poll_tails() and row['tail'].model.events == 12,
          row['tail'].model.events)
    check("the live line moved on (the routing stage, the bus)",
          row['live'] != live and ('buses finished' in row['live'] or 'U1:U2' in row['live']), row['live'])

    print('4. the step ends')
    with open(events, 'a') as f:
        f.write(''.join(json.dumps(e) + '\n' for e in evs[12:]))
    tab._handle_line(line('AUTOROUTE_STEP', dict(run_ev, state='done', rc=0, seconds=40.0,
                                                 result='1/1 bus(es) handed on routed', t=t0 + 40)))
    m = row['tail'].model
    check("the last lines read and nothing of the step left running",
          m.events == len(evs) and not any(r['state'] == 'running' for r in m.rows.values()), m.events)
    tab._refresh_list()
    check("the Result column back to the step's result",
          tab.step_list.GetItemText(0, 4).startswith('1/1 bus(es) handed on routed'),
          tab.step_list.GetItemText(0, 4))

    print('5. a bus step that cannot run')
    tab._reset_run_state()
    tab._handle_line(line('AUTOROUTE_PLAN', dict(plan, bus=dict(
        bus, missing='the bus step needs ortools: not installed; install with: "py" -m pip install ortools'))))
    check("the plan says NOT RUN with the install line",
          'NOT RUN: the bus step needs ortools' in tab.plan_text.GetValue() and 'pip install' in
          tab.plan_text.GetValue())
    frame.Destroy()
    print(f"VERDICT: {'FAIL (' + str(len(FAILS)) + '): ' + '; '.join(FAILS) if FAILS else 'PASS'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
