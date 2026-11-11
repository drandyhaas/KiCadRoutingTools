#!/usr/bin/env python3
"""#1807: a route step records the nets it lost and gained, and the next
step's gate recognises a RIP CYCLE.

Each route.py step is its own process. A retry that broke what the step
before it connected could not tell: the gate's lists were printed and
forgotten, so a rejected trade was run again step after step with the same
report. The gate now appends each step's trade to the board's records
(`kicad_routing_tools.route_laps`, a rejected step's too), compares the next
step's trade with the newest ACCEPTED one (and with any identical trade
rejected since), prints `RIP CYCLE:` naming the contested set, and adds
`rip_cycle` to `JSON_IMPROVEMENT_GATE`. The summary's `rip_attackers` is the
structured twin of `preexisting_rips`' "[was blocking ...]" text.

A. the comparison on hand-built records.
B. the record's file round trip (replaced, not merged).
C. a chain of F.Cu-only laps on an in-repo board, each retry naming the nets
   still open, where the third step breaks two nets the second connected.
"""
import json
import os
import re
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, HERE)

from improvement_gate import (rip_cycle, lap_entry, lap_history,  # noqa: E402
                              format_rip_cycle, LAP_KEY)
from run_utils import check as run_check, evidence  # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def cmp(lost, gained):
    return {'lost': sorted(lost), 'gained': sorted(gained)}


print("A. the comparison")
lap2 = lap_entry(cmp([], ['/DTR', '/EN', 'C4']), 'accept')
lap3 = cmp(['/DCOM', '/DTR', '/EN'], ['C1'])
cyc = rip_cycle([lap2], lap3, {'/DTR': ['/+3.3V']})
check("a step that breaks nets the step before it connected is a cycle",
      cyc is not None and cyc['undone'] == ['/DTR', '/EN'], str(cyc))
check("...its contested set is this step's whole trade",
      cyc and cyc['contested'] == ['/DCOM', '/DTR', '/EN', 'C1'], str(cyc))
check("...with who ripped whom", cyc and cyc['attackers'] == {'/DTR': ['/+3.3V']})
line = format_rip_cycle(cyc) if cyc else ''
check("...on one RIP CYCLE line naming them",
      line.startswith('RIP CYCLE: ') and 'C1' in line and '/DTR' in line, line)
rej3 = lap_entry(lap3, 'reject')
cyc = rip_cycle([lap2, rej3], lap3)
check("the same trade after its rejection: still against the ACCEPTED step, "
      "and named a repeat", cyc and cyc['undone'] == ['/DTR', '/EN']
      and cyc['repeats_rejected'], str(cyc))
check("control: no record, no cycle", rip_cycle([], lap3) is None)
check("control: a trade that meets nothing the last step did is no cycle",
      rip_cycle([lap2], cmp(['/X'], ['/Y'])) is None)
check("control: re-connecting what the last step broke, breaking nothing, "
      "is a repair", rip_cycle([lap_entry(cmp(['/A'], ['/B']), 'accept')],
                               cmp([], ['/A'])) is None)
check("...but doing it by breaking another is a cycle",
      (rip_cycle([lap_entry(cmp(['/A'], ['/B']), 'accept')],
                 cmp(['/C'], ['/A'])) or {}).get('redone') == ['/A'])
check("a step that traded nothing records nothing",
      lap_entry(cmp([], []), 'accept') is None)
check("malformed records are ignored",
      lap_history({LAP_KEY: [{'lost': 'x'}, 3, lap2]}) == [lap2])

print("B. the record's file round trip")
from protected_nets import persist_lap_history  # noqa: E402
import tool_records  # noqa: E402
tmp = tempfile.mkdtemp(prefix='lap1807_')
try:
    pro = os.path.join(tmp, 'b.kicad_pro')
    with open(pro, 'w', encoding='utf-8') as f:
        json.dump({'meta': {'version': 1}}, f)
    persist_lap_history(pro, [lap2], verbose=False)
    persist_lap_history(pro, [lap2, rej3], verbose=False)
    got = lap_history(tool_records.effective_namespace(pro))
    check("the second write replaces the list (no merge)", got == [lap2, rej3],
          str(got))
    check("no project, no record (and no error)",
          persist_lap_history(os.path.join(tmp, 'none.kicad_pro'), [lap2],
                              verbose=False) is False)
finally:
    shutil.rmtree(tmp, ignore_errors=True)

print("C. a chain of F.Cu-only retry laps")
SRC = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
evidence(SRC, 'the in-repo board the chain routes')
PY = [sys.executable, '-X', 'utf8']


def gate(out):
    m = re.findall(r'JSON_IMPROVEMENT_GATE: (\{.*\})', out)
    return json.loads(m[-1]) if m else None


def open_nets(board):
    """The nets check_connected reports unrouted or broken (it exits 1 when
    there are any, so the reason is asserted, not the code)."""
    r = run_check(PY + [os.path.join(ROOT, 'py_router', 'check_connected.py'),
                        board], refuse='Disconnected components', code=1)
    names = []
    for ln in r.stdout.splitlines():
        m = (re.match(r'^  ([^ ].*) \(net \d+\):', ln)
             or re.match(r'^    ([^ ].*) \(\d+ pads\)$', ln))
        if m:
            names.append(m.group(1))
    return names


tmp = tempfile.mkdtemp(prefix='lap1807_')
try:
    b = [os.path.join(tmp, f'lap{i}.kicad_pcb') for i in range(4)]
    run_check(PY + [os.path.join(ROOT, 'py_router', 'copy_board.py'), SRC, b[0]],
              accept=True)
    route = PY + [os.path.join(ROOT, 'py_router', 'route.py')]
    run_check(route + [b[0], b[1], '--nets', '*', '--layers', 'F.Cu'],
              accept=True, timeout=1200)
    logs, gates = {}, {}
    for i in (2, 3):
        nets = open_nets(b[i - 1])
        check(f"lap {i} has open nets to retry", bool(nets))
        r = run_check(route + [b[i - 1], b[i], '--nets'] + nets
                      + ['--layers', 'F.Cu'], accept=True, timeout=1200)
        logs[i], gates[i] = r.stdout, gate(r.stdout)
        check(f"lap {i} printed its gate", gates[i] is not None)
    rec2 = lap_history(tool_records.effective_namespace(b[2]))
    check("after lap 2 the board records its trade",
          bool(rec2) and rec2[-1]['gained'] == gates[2]['gained']
          and rec2[-1]['lost'] == gates[2]['lost'], str(rec2[-1:]))
    undone = sorted(set(gates[3]['lost']) & set(gates[2]['gained']))
    check("the fixture still reproduces: lap 3 breaks nets lap 2 connected",
          bool(undone), f"lap 2 gained {gates[2]['gained']}, lap 3 lost "
          f"{gates[3]['lost']}")
    cyc = gates[3].get('rip_cycle') or {}
    check("lap 3's gate JSON carries the cycle, naming exactly those nets",
          cyc.get('undone') == undone, str(cyc))
    check("...and the console says RIP CYCLE", 'RIP CYCLE: ' in logs[3])
    check("control: lap 2's gate names no cycle (it broke nothing lap 1 "
          "connected)", not set(gates[2]['lost'])
          and 'rip_cycle' not in gates[2] and 'RIP CYCLE' not in logs[2])
    check("control: the verdicts are the gate's own (lap 2 accept, lap 3 "
          "reject)", (gates[2]['verdict'], gates[3]['verdict'])
          == ('accept', 'reject'), f"{gates[2]['verdict']}, {gates[3]['verdict']}")
    rec3 = lap_history(tool_records.effective_namespace(b[3]))
    check("the rejected lap 3 records its trade beside the reverted board",
          bool(rec3) and rec3[-1]['verdict'] == 'reject'
          and rec3[-1]['lost'] == gates[3]['lost'], str(rec3[-1:]))
    # rip_attackers agrees with the text of preexisting_rips (its last entry)
    ok, seen = True, 0
    for out in logs.values():
        for s in re.findall(r'JSON_SUMMARY: (\{.*\})', out):
            d = json.loads(s)
            for victim, text in (d.get('preexisting_rips') or {}).items():
                m = re.search(r"\[was blocking '(.*)'\]$", text)
                if m:
                    seen += 1
                    ok &= (d.get('rip_attackers') or {}).get(victim, [None])[-1] \
                        == m.group(1)
    check("rip_attackers ends with the net preexisting_rips names", ok and seen > 0,
          f"{seen} victim(s) seen")
finally:
    shutil.rmtree(tmp, ignore_errors=True)

print()
if failures:
    print(f"{len(failures)} FAILURE(S): " + "; ".join(failures))
    sys.exit(1)
print("All #1807 rip-cycle record checks passed")
