#!/usr/bin/env python3
"""A class-pair .kicad_dru rule sets a diff pair's coupling gap (#1526).

A pair class whose clearance is wider than its pitch, relaxed between members
of the class by the board's own rule:

    (rule "csi intra gap" (constraint clearance (min 0.127mm))
      (condition "A.NetClass == 'CSI' && B.NetClass == 'CSI'"))

KiCad grades P against N under that rule (it replaces the class value), and
so did check_drc and the router's own pair_clearance (the #1491 scoped
channel). route_diff alone raised the coupling gap to the class clearance
(#530), and could only ever raise it at a scoped rule (`_pair_rule_gap`), so
a 0.25 class could not leave fine-pitch pins its rules allow. Now a scoped
rule that is what KiCad grades the pair's tracks at, on every layer the pair
may use, floors the gap alone -- the channel a #1293 coupled-pair rule takes
(route_diff.coupled_pair_floors). KiCad's coupling check still grades the gap
itself at the board minimum clearance, which an explicit clearance rule does
not lower, so the gap is floored there too.

The fixture is qfn_csi_underpad_diff (pairs CSI_CLK, CSI_D0, 4 layers):
Default 0.127, class CSI (CSI_*) at clearance and pair gap 0.25 -- equal, so
the class's own "(diff pair)" rule (#1542) does not apply -- and the rule
above. Measured on kicad-cli 10.0.6: 0.127-built pairs grade 0 P<->N errors
with the rule, and 22 diff_pair_gap_out_of_range under a 0.2 board minimum.

Checks:
  A. --diff-pair-gap 0.127 under the rule: both pairs built at 0.127 (were
     0.25), the rule named, no #530 raise; check_drc clean; KiCad clean
  B. without the rule: raised to the class clearance 0.25 as before
  C. a 0.2 board minimum: built at 0.2, KiCad clean (no gap out of range)

    python3 tests/test_1526_scoped_pair_gap.py
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import evidence, tool                  # noqa: E402

FAILS = []
RULE = ('(version 1)\n(rule "csi intra gap" (constraint clearance (min 0.127mm))\n'
        "  (condition \"A.NetClass == 'CSI' && B.NetClass == 'CSI'\"))\n")


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def _cls(name, clr, prio):
    return {'name': name, 'clearance': clr, 'track_width': 0.15, 'via_diameter': 0.6,
            'via_drill': 0.3, 'diff_pair_gap': clr, 'diff_pair_width': 0.15,
            'diff_pair_via_gap': 0.25, 'microvia_diameter': 0.3, 'microvia_drill': 0.1,
            'priority': prio, 'bus_width': 12, 'wire_width': 6, 'line_style': 0,
            'pcb_color': 'rgba(0, 0, 0, 0.000)', 'schematic_color': 'rgba(0, 0, 0, 0.000)'}


def stage(d, stem, rule, min_clearance=0.0):
    pcb = os.path.join(d, stem + '.kicad_pcb')
    shutil.copy(os.path.join(ROOT, 'kicad_files', 'qfn_csi_underpad_diff.kicad_pcb'), pcb)
    pro = {'board': {'design_settings': {'rules': {'min_clearance': min_clearance,
                                                   'min_track_width': 0.0}}},
           'net_settings': {'meta': {'version': 4},
                            'classes': [_cls('Default', 0.127, 2147483647),
                                        _cls('CSI', 0.25, 0)],
                            'netclass_patterns': [{'netclass': 'CSI', 'pattern': 'CSI_*'}]},
           'meta': {'filename': stem + '.kicad_pro', 'version': 3}}
    with open(os.path.join(d, stem + '.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(pro, fh, indent=2)
    if rule:
        with open(os.path.join(d, stem + '.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(rule)
    return evidence(pcb, 'staged board')


def route(d, stem, rule, **kw):
    """route_diff.py as the issue ran it; (log, {pair: (outcome, gap_mm)},
    routed board)."""
    src = stage(d, stem, rule, **kw)
    out = os.path.join(d, stem + '_o.kicad_pcb')
    r = subprocess.run([sys.executable, '-X', 'utf8', tool('route_diff.py'), src, out,
                        '--nets', 'CSI_*', '--diff-pair-gap', '0.127'],
                       cwd=ROOT, capture_output=True, text=True, timeout=900)
    if r.returncode != 0:
        raise AssertionError(f'route_diff.py failed: {r.stderr[-400:]}')
    summ = [ln for ln in r.stdout.splitlines() if ln.startswith('JSON_SUMMARY: ')]
    reports = json.loads(summ[-1][len('JSON_SUMMARY: '):])['pair_reports'] if summ else []
    pairs = {p['pair']: (p['outcome'], (p.get('copper') or {}).get('gap_mm'))
             for p in reports}
    return r.stdout, pairs, evidence(out, 'routed board')


def coupled_at(pairs, mm):
    return set(pairs) == {'CSI_CLK', 'CSI_D0'} and all(
        o == 'coupled' and g is not None and abs(g - mm) < 0.002
        for o, g in pairs.values())


def check_drc_pn(path):
    rep = path + '.krt.json'
    r = subprocess.run([sys.executable, '-X', 'utf8', tool('check_drc.py'), path,
                        '--json', rep], cwd=ROOT, capture_output=True, text=True,
                       timeout=600)
    if r.returncode not in (0, 1):
        raise AssertionError(f'check_drc failed: {r.stderr[-400:]}')
    with open(evidence(rep, 'check_drc report'), encoding='utf-8') as fh:
        items = json.load(fh)['items']
    return [x for x in items if x.get('net1') and x.get('net2')
            and re.sub(r'_[PN]$', '', x['net1']) == re.sub(r'_[PN]$', '', x['net2'])]


def kicad(path):
    from kicad_oracle import find_kicad_cli
    cli = find_kicad_cli(warn=False)
    if not cli:
        print("  SKIP: no kicad-cli found; KiCad's own grade is unchecked here")
        return None
    rep = path + '.drc.json'
    subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all',
                    '--all-track-errors', '-o', rep, path],
                   capture_output=True, text=True, timeout=600)
    with open(evidence(rep, 'kicad-cli DRC report'), encoding='utf-8') as fh:
        k = json.load(fh)
    return [x['description'] for x in k['violations']
            if x['type'] in ('clearance', 'diff_pair_gap_out_of_range')]


def main():
    with tempfile.TemporaryDirectory(prefix='t1526_') as d:
        print('A. the class-pair rule')
        log, pairs, out = route(d, 'rule', RULE)
        named = [ln.strip() for ln in log.splitlines()
                 if '#1526' in ln and "graded at 0.127mm by the scoped rule 'csi intra gap'" in ln]
        check('the rule is named for both pairs', len(named) == 2, f'{named}')
        check('no #530 raise to the class clearance', not re.search(r'#530: CSI', log))
        check('both pairs couple at the rule, 0.127 mm (were 0.25)',
              coupled_at(pairs, 0.127), f'{pairs}')
        v = check_drc_pn(out)
        check('check_drc: no P<->N violation', not v, f'{len(v)}')
        bad = kicad(out)
        if bad is not None:
            check('KiCad: no clearance or pair-gap violation', not bad, f'{bad[:3]}')

        print('B. no rule')
        log, pairs, out = route(d, 'none', None)
        raised = re.findall(r'#530: CSI_\w+: coupling gap 0\.127 mm raised to its '
                            r'net-class clearance 0\.25', log)
        built = [g for o, g in pairs.values() if o == 'coupled']
        check('raised to the class clearance 0.25 as before (a pair that '
              'cannot leave its pins at it fails)',
              len(raised) == 2 and '#1526' not in log and built
              and all(abs(g - 0.25) < 0.002 for g in built), f'{raised} {pairs}')

        print('C. a 0.2 board minimum')
        log, pairs, out = route(d, 'bmin', RULE, min_clearance=0.2)
        check('floored at the diff_pair_gap minimum 0.2, which is named',
              coupled_at(pairs, 0.2) and 'the diff_pair_gap minimum' in log, f'{pairs}')
        bad = kicad(out)
        if bad is not None:
            check('KiCad: no clearance or pair-gap violation', not bad, f'{bad[:3]}')


if __name__ == '__main__':
    main()
    print()
    if FAILS:
        print(f'FAILED: {len(FAILS)} check(s)')
        for f in FAILS:
            print('  - ' + f)
        sys.exit(1)
    print('ALL PASS')
