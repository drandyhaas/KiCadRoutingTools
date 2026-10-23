#!/usr/bin/env python3
"""tests/stress/placement_survey.py: the classifier behind placement-lessons.md's group priors.

The survey's numbers are only as good as what it decides a part IS, and every
rule below was added after a spot-check over the stress corpus caught a real
misreading. Each check pins one of them, so a later edit cannot quietly undo it:

  1. a net-name voltage orients a regulator whose pins carry no function, and a
     dotted name (`+3.3V`) is a supply rail -- missing it dropped every decap
     on a dotted rail from the survey;
  2. a regulator whose net names do not say which way it points gets NO input
     (an unnamed `Net-(C1-Pad1)` beside `/+3.3V`), never a guess;
  3. a footprint keyword never makes a diode a connector (`D_SMA` is a diode
     package, not an SMA jack);
  4. the `board` and `report` modes run end to end on a tracked board
     (kicad_files/esp_prog.kicad_pcb), and the reproduce commands placement-lessons.md
     cites run VERBATIM on a one-board corpus -- test_431 skips tools under
     tests/, so nothing else holds that recipe to the real parser.

Runs in a few seconds, no corpus needed.
"""
import json
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
TOOL = os.path.join(ROOT, 'tests', 'stress', 'placement_survey.py')
BOARD = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
import placement_survey as ps  # noqa: E402

results = []


def ok(name, cond, detail=''):
    results.append(bool(cond))
    print(f"  {'ok  ' if cond else 'FAIL'} {name}{(' -- ' + detail) if detail and not cond else ''}")


def test_voltages_and_rails():
    cases = {'+5V': 5.0, '/+3.3V': 3.3, '+3V3': 3.3, '1V8': 1.8, '-12V': 12.0,
             'VBUS': 5.0, '/VIN': 99.0, 'GND': None, 'Net-(C1-Pad1)': None}
    for name, want in cases.items():
        got = ps.volts(name)
        ok(f"volts({name!r}) == {want}", got == want, f"got {got}")
    for name in ('/+3.3V', '+1.8V', '+3V3', 'VCC', '/VDD_IO', 'P1V8', '-12V'):
        ok(f"{name!r} reads as a supply rail", ps.SUPPLY_NET.search(name))
    for name in ('GND', 'Net-(C1-Pad1)', '/D_P', '/USB_DN'):
        ok(f"{name!r} is not a supply rail", not ps.SUPPLY_NET.search(name))


def _fp(name, npads, layers=('F.Cu',)):
    pads = [SimpleNamespace(pad_type='smd', layers=list(layers)) for _ in range(npads)]
    return SimpleNamespace(footprint_name=name, pads=pads)


def test_connector_exclusions():
    ok("a D_SMA diode is not a connector",
       not ps.is_connector('D201', _fp('Diode_SMD:D_SMA', 2), None))
    ok("an SMA jack is a connector",
       ps.is_connector('J3', _fp('Connector_Coaxial:SMA_Amphenol_132134', 3), None))
    ok("a crystal is not a connector",
       not ps.is_connector('X1', _fp('Crystal:Crystal_SMD_3225-4Pin', 4), None))
    ok("a test point is not a connector",
       not ps.is_connector('TP1', _fp('TestPoint:TestPoint_Pad_D1.0mm', 1), 'testpoint'))


def test_board_and_report_cli():
    r = subprocess.run([sys.executable, '-X', 'utf8', TOOL, 'board', BOARD],
                       capture_output=True, text=True, timeout=300)
    lines = r.stdout.strip().splitlines()
    ok("board mode prints a JSON row", r.returncode == 0 and lines,
       f"rc={r.returncode} stderr={r.stderr[-300:]}")
    if not lines:
        return
    row = json.loads(lines[-1])
    ok("the row is not an error row", 'error' not in row, row.get('error', ''))
    cond = {c['ref']: c for c in row.get('cond', [])}
    ok("esp_prog U2 (ME6210) is a linear regulator",
       cond.get('U2', {}).get('kind') == 'linear', json.dumps(cond)[:300])
    ok("U2's direction is NOT guessed: Net-(C1-Pad1) vs /+3.3V names no input",
       cond.get('U2', {}).get('vin') == [] and 'entry' not in cond.get('U2', {}),
       json.dumps(cond.get('U2'))[:300])
    xt = {c['ref']: c for c in row.get('crystals', [])}
    ok("crystal Y1 serves the CH340 (U1)", xt.get('Y1', {}).get('ic') == 'U1',
       json.dumps(xt)[:200])
    ok("the decap on the dotted /+3.3V rail is counted", row.get('decap_n') == 1,
       f"decap_n={row.get('decap_n')}")
    conns = {c['ref'] for c in row.get('conns', [])}
    ok("USB1, CON1, CON2 are the connectors", conns == {'USB1', 'CON1', 'CON2'},
       str(sorted(conns)))
    with tempfile.TemporaryDirectory() as td:
        rows = os.path.join(td, 'rows.jsonl')
        with open(rows, 'w') as f:
            f.write(lines[-1] + '\n')
        rp = subprocess.run([sys.executable, '-X', 'utf8', TOOL, 'report', rows],
                            capture_output=True, text=True, timeout=120)
        ok("report mode rolls a row up", rp.returncode == 0 and
           'POWER CONDITIONERS: 1 on 1 boards' in rp.stdout,
           f"rc={rp.returncode} {rp.stdout[:200]} {rp.stderr[-300:]}")


def test_cited_commands_run():
    """The reproduce commands placement-lessons.md prints, run VERBATIM on a one-board
    corpus. test_431 does not hold a tests/ tool's flags to its parser, so this
    is what keeps the page's recipe runnable."""
    doc = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement', 'references',
                       'placement-lessons.md')
    cmds = [shlex.split(line.strip()) for line in open(doc, encoding='utf-8')
            if line.strip().startswith('python3 tests/stress/placement_survey.py')]
    ok("placement-lessons.md cites a run and a report command",
       [c[2] for c in cmds] == ['run', 'report'], str(cmds))
    if len(cmds) != 2:
        return
    with tempfile.TemporaryDirectory() as td:
        os.makedirs(os.path.join(td, 'boards_set1'))
        shutil.copy(BOARD, os.path.join(td, 'boards_set1', 'esp_prog.kicad_pcb'))
        for argv in cmds:
            argv = [sys.executable, os.path.join(ROOT, argv[1])] + argv[2:]
            if argv[2] == 'run':
                argv += ['--stress-dir', td]
            r = subprocess.run(argv, cwd=td, capture_output=True, text=True, timeout=300)
            ok(f"`{argv[2]}` as cited exits 0", r.returncode == 0,
               f"rc={r.returncode} {r.stderr[-300:]}")
        ok("the cited pair rolls the one board up",
           'POWER CONDITIONERS: 1 on 1 boards' in r.stdout, r.stdout[:200])


def main():
    print("placement_survey: the classifier behind placement-lessons.md's group priors")
    test_voltages_and_rails()
    test_connector_exclusions()
    test_board_and_report_cli()
    test_cited_commands_run()
    passed = sum(results)
    print(f"\n{passed}/{len(results)} checks passed")
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
