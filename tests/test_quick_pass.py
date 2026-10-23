#!/usr/bin/env python3
"""route.py --quick: a placement review's first round (level 1).

The main pass runs; the recovery tail does not -- the net rescue, the
terminal escalation, the plane finalize and the reconciliation laps -- and
the run says so (JSON_SUMMARY `quick`), because its board is a diagnosis,
never one to ship. On a tracked board, in-process, with the tail's entry
points spied on:
  1. with quick=1 the rescue and the escalation are never called, and the
     summary names all four skipped stages;
  2. the control, the same run without it, calls both and writes no
     `quick` key -- so the spies can see a call, and (1) is not vacuous;
  3. the CLI flag reaches the engine (`route.py --quick --json-out`).
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import net_rescue  # noqa: E402
import route  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
results = []


def ok(name, cond, detail=''):
    results.append(cond)
    print(f"  {'ok  ' if cond else 'FAIL'} {name}" + (f" -- {detail}" if not cond and detail else ''))


def _run(td, tag, quick):
    calls = []
    orig = (net_rescue.rescue_failed_nets, net_rescue.terminal_geometry_escalation)

    def spy(name, fn):
        def wrapped(*a, **k):
            calls.append(name)
            return fn(*a, **k)
        return wrapped
    net_rescue.rescue_failed_nets = spy('rescue', orig[0])
    net_rescue.terminal_geometry_escalation = spy('escalation', orig[1])
    try:
        js = os.path.join(td, f'{tag}.json')
        nets = sorted(n.name for nid, n in parse_kicad_pcb(BOARD).nets.items()
                      if nid and n.name)
        route.batch_route(BOARD, os.path.join(td, f'{tag}.kicad_pcb'), nets,
                          quick=quick, json_out=js)
        return calls, json.load(open(js, encoding='utf-8'))
    finally:
        net_rescue.rescue_failed_nets, net_rescue.terminal_geometry_escalation = orig


def main():
    print("route.py --quick: the main pass without the recovery tail")
    with tempfile.TemporaryDirectory() as td:
        calls, doc = _run(td, 'quick', 1)
        ok("quick pass: no rescue, no escalation", calls == [], str(calls))
        qp = doc.get('quick') or {}
        ok("quick pass: the summary names the four skipped stages",
           qp.get('skipped') == ['net_rescue', 'terminal_escalation',
                                 'plane_finalize', 'reconciliation'], str(qp))
        calls, doc = _run(td, 'full', 0)
        ok("control: the full run calls the rescue and the escalation",
           'rescue' in calls and 'escalation' in calls, str(calls))
        ok("control: the full run writes no quick key", 'quick' not in doc)
        js = os.path.join(td, 'cli.json')
        r = subprocess.run([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'route.py'),
                            BOARD, os.path.join(td, 'cli.kicad_pcb'), '--nets', '*',
                            '--quick', '--json-out', js],
                           capture_output=True, text=True, timeout=300)
        ok("CLI: --quick exits 0", r.returncode == 0, r.stderr[-300:])
        ok("CLI: --quick reaches the engine",
           os.path.isfile(js) and 'quick' in json.load(open(js, encoding='utf-8')))
    print(f"\n{sum(results)}/{len(results)} checks passed")
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
