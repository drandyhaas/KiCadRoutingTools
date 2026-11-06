#!/usr/bin/env python3
"""#1733: a `(severity ignore)` .kicad_dru clearance rule binds, and reports nothing.

No clearance channel read such a rule (kicad_dru skipped it with "severity
ignore, skipped"), so check_drc graded the pairs it binds at the class or an
earlier rule and counted them, where KiCad's last-match resolution lands on
the ignore rule and reports nothing. Now the severity pass asks the resolver
which clearance rule binds each clearance row; when that is an ignore rule
the row is listed SEVERITY IGNORE (accepted, not counted).

The board: parallel 0.2 mm track pairs 0.15 mm apart (/A-/B and /C-/D on
F.Cu, /G-/H on B.Cu), a pad pair of two parts 0.15 mm apart (/P-/Q), and a
crossing (/E-/F), all under a 0.2 mm Default class. Arms, with the clearance
pairs KiCad reports (kicad-cli 10.0.6, re-measured here when it is installed):
  none    no rule                                         AB CD GH PQ
  alone   ignore rule on /A and /E                        CD GH PQ
  over    error rule on /C, then an ignore rule on /C     AB GH PQ
  under   ignore rule on /C, then an error rule on /C     AB CD GH PQ
  layer   error B.Cu layer rule, then an ignore one       AB CD PQ
  pads    ignore rule on /P                               AB CD GH
The /E-/F crossing touches: KiCad's ignore rule silences it too, but check_drc
keeps counting copper below the physical fab floor whatever the severity
(#1626), so it is asserted counted in every arm.

    python3 tests/test_1733_ignore_clearance_rule.py
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FAILS = []
NETS = ['/A', '/B', '/C', '/D', '/E', '/F', '/G', '/H', '/P', '/Q']
TRACKS = [('/A', 'F.Cu', 100.0), ('/B', 'F.Cu', 100.35), ('/C', 'F.Cu', 103.0),
          ('/D', 'F.Cu', 103.35), ('/E', 'F.Cu', 106.0), ('/F', 'F.Cu', 106.0),
          ('/G', 'B.Cu', 109.0), ('/H', 'B.Cu', 109.35)]
PARTS = [('R1', 120.0, 104.0, '/P'), ('R2', 120.65, 104.0, '/Q'),
         ('R3', 105.0, 106.0, '/E'), ('R4', 110.0, 104.85, '/F')]


def nm(*ns):
    return ' || '.join(f"A.NetName == '{n}'" for n in ns)


ARMS = {
    'none': ([], {'AB', 'CD', 'GH', 'PQ'}),
    'alone': ([f'(rule "q" (severity ignore) (condition "{nm("/A", "/E")}") '
               '(constraint clearance (min 0.3mm)))'], {'CD', 'GH', 'PQ'}),
    'over': ([f'(rule "e" (condition "{nm("/C")}") (constraint clearance (min 0.3mm)))',
              f'(rule "q" (severity ignore) (condition "{nm("/C")}") '
              '(constraint clearance (min 0.1mm)))'], {'AB', 'GH', 'PQ'}),
    'under': ([f'(rule "q" (severity ignore) (condition "{nm("/C")}") '
               '(constraint clearance (min 0.1mm)))',
               f'(rule "e" (condition "{nm("/C")}") (constraint clearance (min 0.3mm)))'],
              {'AB', 'CD', 'GH', 'PQ'}),
    'layer': (['(rule "eb" (layer B.Cu) (constraint clearance (min 0.3mm)))',
               '(rule "qb" (severity ignore) (layer B.Cu) (constraint clearance (min 0.1mm)))'],
              {'AB', 'CD', 'PQ'}),
    'pads': ([f'(rule "qp" (severity ignore) (condition "{nm("/P")}") '
              '(constraint clearance (min 0.3mm)))'], {'AB', 'CD', 'GH'}),
}
#: arm -> the rule each silenced pair is listed under
SILENCED = {'alone': {'AB': 'q'}, 'over': {'CD': 'q'}, 'layer': {'GH': 'qb'},
            'pads': {'PQ': 'qp'}}


def ok(name, cond, detail=''):
    print(('  [ok] ' if cond else '  [FAIL] ') + name
          + ('' if cond else f': {detail}'))
    if not cond:
        FAILS.append(name)


def pair(n1, n2):
    return ''.join(sorted(x.strip('/') for x in (n1, n2)))


def build(d, rules):
    os.makedirs(d, exist_ok=True)
    num = {n: i + 1 for i, n in enumerate(NETS)}
    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
         ' (setup (pad_to_mask_clearance 0))\n (net 0 "")\n'
         + ''.join(f' (net {num[n]} "{n}")\n' for n in NETS))
    for ref, x, y, net in PARTS:
        b += (f'  (footprint "T:{ref}" (layer "F.Cu") (at {x} {y})\n'
              f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") (hide yes) '
              f'(effects (font (size 1 1) (thickness 0.15))))\n    (attr smd)\n'
              f'    (pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "F.Cu") '
              f'(net {num[net]} "{net}"))\n  )\n')
    for k, (net, layer, y) in enumerate(TRACKS):
        a, e = ((110, y - 1.15), (110, y + 0.85)) if net == '/F' else ((105, y), (115, y))
        b += (f'  (segment (start {a[0]} {a[1]}) (end {e[0]} {e[1]}) (width 0.2) '
              f'(layer "{layer}") (net {num[net]}) '
              f'(uuid "00000000-0000-0000-0000-0000000000{k:02d}"))\n')
    for x1, y1, x2, y2 in ((100, 95, 130, 95), (130, 95, 130, 115),
                           (130, 115, 100, 115), (100, 115, 100, 95)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    pcb = os.path.join(d, 'b.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(b + ')\n')
    cl = dict(name='Default', clearance=0.2, track_width=0.2, via_diameter=0.6,
              via_drill=0.3, wire_width=6, bus_width=12, line_style=0,
              priority=2147483647)
    with open(os.path.join(d, 'b.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {
                       'rules': {'min_clearance': 0.1, 'min_track_width': 0.1},
                       'rule_severities': {}}},
                   'meta': {'filename': 'b.kicad_pro', 'version': 3},
                   'net_settings': {'classes': [cl], 'meta': {'version': 4}}},
                  f, indent=1)
    if rules:
        with open(os.path.join(d, 'b.kicad_dru'), 'w', encoding='utf-8') as f:
            f.write('(version 1)\n' + '\n'.join(rules) + '\n')
    return pcb


def check_drc(pcb):
    js = pcb + '.drc.json'
    r = run_utils.check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), pcb,
                              '--json', js], code=1)
    with open(run_utils.evidence(js, 'check_drc --json'), encoding='utf-8') as f:
        return json.load(f), r.stdout


def main():
    from kicad_oracle import find_kicad_cli
    cli = find_kicad_cli(warn=False)
    tmp = tempfile.mkdtemp(prefix='t1733_')
    try:
        for arm, (rules, kicad_pairs) in ARMS.items():
            print(f'== {arm}')
            pcb = build(os.path.join(tmp, arm), rules)
            doc, out = check_drc(pcb)
            counted = {pair(v['net1'], v['net2']) for v in doc['items']
                       if v.get('type') in ('segment-segment', 'pad-pad')
                       and not v.get('accepted')}
            ok(f'{arm}: check_drc counts the clearance pairs KiCad reports',
               counted == kicad_pairs, (sorted(counted), sorted(kicad_pairs)))
            quiet = {pair(v['net1'], v['net2']): v.get('severity_source') for v in doc['items']
                     if v.get('accepted') == 'severity-ignore'}
            want = {p: f"rule '{r}'" for p, r in SILENCED.get(arm, {}).items()}
            ok(f'{arm}: the silenced pairs are listed under the ignore rule', quiet == want,
               quiet)
            ok(f'{arm}: the crossing still counts (copper below the fab floor)',
               any(v.get('type') == 'segment-crossing' and not v.get('accepted')
                   for v in doc['items']), [v.get('type') for v in doc['items']])
            if cli:
                kd = os.path.join(tmp, arm + '_k')
                os.makedirs(kd, exist_ok=True)
                subprocess.run(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'), pcb,
                                     os.path.join(kd, 'b.kicad_pcb')],
                               capture_output=True, timeout=120, cwd=REPO)
                kj = os.path.join(kd, 'k.json')
                subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all',
                                '-o', kj, os.path.join(kd, 'b.kicad_pcb')],
                               capture_output=True, timeout=600)
                with open(run_utils.evidence(kj, 'kicad-cli drc json'), encoding='utf-8') as f:
                    kv = json.load(f)['violations']
                got = {pair(*[re.search(r'\[(.*?)\]', i['description']).group(1)
                              for i in v['items']]) for v in kv if v['type'] == 'clearance'}
                ok(f'{arm}: KiCad reports what was measured', got == kicad_pairs,
                   sorted(got))
        if not cli:
            print("  (kicad-cli not found: KiCad's verdicts are not re-measured)")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f'FAILED: {len(FAILS)}: {FAILS}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
