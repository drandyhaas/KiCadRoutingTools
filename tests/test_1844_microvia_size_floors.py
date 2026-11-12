#!/usr/bin/env python3
"""#1844: check_drc grades a micro via at the board's micro via minimums.

A via typed micro was graded at Board Setup's through-via diameter and
through-hole drill, so a micro via KiCad passes was reported as VIA-SIZE and
VIA-DRILL-SIZE. `DesignRules.via_item` now carries the via's type, and a micro
via resolves its diameter and drill as KiCad does.

The fixture (tests/fixtures/1844) is synthetic: Board Setup min via 0.4,
min through hole 0.15, min micro via 0.25, min micro drill 0.1; a micro via
`UV` 0.25/0.1 on F.Cu-In1.Cu and a through via `TV` 0.46/0.2.

Every row of `ROWS` is one variant of that board with the size violations
kicad-cli 10.0.8 reports on it (measured; via_diameter / drill / annular):
  * the drill minimum of a micro via is Board Setup's `min_microvia_drill`,
    0.1 mm when the project does not carry the key;
  * its diameter minimum is its net class's `microvia_diameter`, which
    replaces Board Setup's `min_microvia_diameter`, above or below it;
  * the annular ring minimum binds a micro via as it does any via;
  * a blind via keeps the through-via minimums.

Checks (no routing):
  1. each row's check_drc size violations are KiCad's;
  2. the through via of the fixture is still graded at 0.4 / 0.15;
  3. `DesignRules.via_item` gives a micro via its type and its own layers;
  4. a micro via's drill finding is governed by KiCad's own severity key for
     it, `microvia_drill_out_of_range`, not the through-hole
     `drill_out_of_range` (both measured on kicad-cli 10.0.8).
"""
import contextlib
import io
import json
import os
import re
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import evidence  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from design_rules import DesignRules  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIX = os.path.join(TESTS, 'fixtures', '1844')
SIZE_TYPES = ('via-size', 'via-drill-size', 'via-annular')

#: name -> (edits to the board text, edits to the project JSON, the size
#: violations kicad-cli 10.0.8 reports as {type: minimum it names})
ROWS = {
    'as drawn': ({}, {}, {}),
    'below the micro minimums': (
        {'uv': (0.2, 0.08)}, {},
        {'via-size': 0.25, 'via-drill-size': 0.1, 'via-annular': 0.07}),
    'class diameter above Board Setup': (
        {'uv': (0.3, 0.12)}, {'class': {'microvia_diameter': 0.33, 'microvia_drill': 0.13}},
        {'via-size': 0.33}),
    'class diameter below Board Setup': (
        {'uv': (0.2, 0.08)}, {'class': {'microvia_diameter': 0.15, 'microvia_drill': 0.05}},
        {'via-drill-size': 0.1, 'via-annular': 0.07}),
    'Board Setup diameter above the class': (
        {}, {'rules': {'min_microvia_diameter': 0.5}}, {}),
    'Board Setup drill raised': (
        {}, {'rules': {'min_microvia_drill': 0.2}}, {'via-drill-size': 0.2}),
    'no micro keys in Board Setup': (
        {'uv': (0.15, 0.05)},
        {'drop_rules': ('min_microvia_diameter', 'min_microvia_drill'),
         'class': {'microvia_diameter': 0.17, 'microvia_drill': 0.06}},
        {'via-size': 0.17, 'via-drill-size': 0.1, 'via-annular': 0.07}),
    'typed blind': (
        {'type': 'blind'}, {}, {'via-size': 0.4, 'via-drill-size': 0.15}),
}

FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def stage(td, name, board_edits, project_edits):
    with open(evidence(os.path.join(FIX, 'via_type_rule.kicad_pcb')), encoding='utf-8') as fh:
        t = fh.read()
    if 'uv' in board_edits:
        size, drill = board_edits['uv']
        t, n = re.subn(r'\(size 0\.25\)(\s*)\(drill 0\.1\)',
                       lambda m: f'(size {size}){m.group(1)}(drill {drill})', t)
        assert n == 1, 'the fixture has one 0.25/0.1 via'
    if 'type' in board_edits:
        assert t.count('(via micro') == 1
        t = t.replace('(via micro', f"(via {board_edits['type']}")
    b = os.path.join(td, name + '.kicad_pcb')
    with open(b, 'w', encoding='utf-8') as fh:
        fh.write(t)
    with open(evidence(os.path.join(FIX, 'via_type_rule.kicad_pro')), encoding='utf-8') as fh:
        proj = json.load(fh)
    rules = proj['board']['design_settings']['rules']
    rules.update(project_edits.get('rules', {}))
    for k in project_edits.get('drop_rules', ()):
        del rules[k]
    proj['net_settings']['classes'][0].update(project_edits.get('class', {}))
    if 'severities' in project_edits:
        proj['board']['design_settings']['rule_severities'] = project_edits['severities']
    with open(os.path.join(td, name + '.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(proj, fh, indent=2)
    return evidence(b, 'the staged board')


def grade(path):
    """check_drc's violation rows for ``path``."""
    import subprocess
    out = path[:-len('.kicad_pcb')] + '.drc.json'
    r = subprocess.run(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), path,
                             '--json', out],
                       capture_output=True, text=True, encoding='utf-8', errors='replace')
    text = (r.stdout or '') + (r.stderr or '')
    if 'Traceback' in text or r.returncode not in (0, 1):
        raise AssertionError(f"BROKEN TEST: check_drc died\n{text[-1500:]}")
    with open(evidence(out, 'the check_drc report'), encoding='utf-8') as fh:
        return json.load(fh)['items'], text


def rows(td):
    print("1. check_drc reports KiCad's size violations on every variant")
    for i, (name, (bed, ped, want)) in enumerate(ROWS.items()):
        items, _ = grade(stage(td, f'r{i}', bed, ped))
        got = {}
        for v in items:
            if v['type'] in SIZE_TYPES and v.get('net1') == 'UV':
                got[v['type']] = round(v.get('min_size') or v.get('min_drill')
                                       or v.get('min_ring'), 4)
        ok(got == want, f"{name}: {got}" + ('' if got == want else f" want {want}"))


def through(td):
    print("2. the through via keeps the through-via minimums")
    path = stage(td, 'tv', {}, {})
    with open(path, encoding='utf-8') as fh:
        t = fh.read()
    t, n = re.subn(r'\(size 0\.46\)(\s*)\(drill 0\.2\)', r'(size 0.39)\1(drill 0.14)', t)
    assert n == 1, 'the fixture has one 0.46/0.2 via'
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(t)
    items, text = grade(path)
    got = {v['type']: round(v.get('min_size') or v.get('min_drill'), 4)
           for v in items if v['type'] in ('via-size', 'via-drill-size')
           and v.get('net1') == 'TV'}
    ok(got == {'via-size': 0.4, 'via-drill-size': 0.15}, f"TV 0.39/0.14: {got}")
    ok(not any(v.get('net1') == 'UV' and v['type'] in SIZE_TYPES for v in items),
       "and the micro via beside it is not flagged")
    ok('micro vias: graded at' in text, "the header says how micro vias are graded")


def item(td):
    print("3. via_item carries the via's type and layers")
    path = stage(td, 'it', {}, {})
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        dr = DesignRules.from_project(pcb, path)
    by = {pcb.nets[v.net_id].name: dr.via_item(v) for v in pcb.vias}
    ok(by['UV'].via_type == 'micro' and by['UV'].other_layers == frozenset({'F.Cu', 'In1.Cu'}),
       f"UV: {by['UV'].via_type} on {sorted(by['UV'].other_layers)}")
    ok(by['TV'].via_type == 'through'
       and by['TV'].other_layers == frozenset(pcb.board_info.copper_layers),
       f"TV: {by['TV'].via_type} on {sorted(by['TV'].other_layers)}")


def severity(td):
    print("4. the micro via drill finding follows microvia_drill_out_of_range")
    raised = {'min_microvia_drill': 0.2}
    for key, counted in (('microvia_drill_out_of_range', False),
                         ('drill_out_of_range', True)):
        path = stage(td, 's_' + key, {}, {'rules': raised, 'severities': {key: 'ignore'}})
        items, text = grade(path)
        got = any(v['type'] == 'via-drill-size' and v.get('net1') == 'UV'
                  and not v.get('accepted') for v in items)
        ok(got == counted, f"{key}: ignore -> the finding is "
                           f"{'counted' if got else 'not counted'}")
        if not counted:
            ok('rule_severities.microvia_drill_out_of_range' in text,
               "and the run names the severity that dropped it")


def main():
    with tempfile.TemporaryDirectory() as td:
        rows(td)
        through(td)
        item(td)
        severity(td)
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
