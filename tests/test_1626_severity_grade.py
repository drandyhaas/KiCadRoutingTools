#!/usr/bin/env python3
"""#1626: check_drc grades each finding at the severity KiCad gives it.

A `.kicad_dru` rule's own `(severity warning)`, or the project's
`rule_severities` set to `warning`, makes KiCad report a WARNING; a project
`ignore` makes KiCad skip the check. check_drc used to count all of them, so
board_score called the board blocking and check_complete INCOMPLETE where
KiCad's own DRC reported no error. Now a warning is listed under WARNINGS
and accepted (`severity-warning`), an ignore is dropped and counted
(`severity-ignore`), and an unset severity stays an error.

Precedence, measured on kicad-cli 10.0.6 with this file's board: a rule's own
severity beats the project's error/warning (rule error + project warning is
an error, rule warning + project error a warning), and a project `ignore`
beats any rule (rule warning + project ignore reports nothing).

Each arm is one /SIG (class HS) track with a via, 0.5 mm from /OTHER, clean
at the 0.2 classes, with a class rule that either forbids the via or asks
0.7 mm. Checks: check_drc's exit and its --json counts, the severity source
on each row, board_score's `drc` and its disclosure, and
`severity_overrides` (kicad_drc_compare's staged-at-error grade) restoring
the count.

    python3 tests/test_1626_severity_grade.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FAILS = []


def ok(name, cond, detail=''):
    print(('  [ok] ' if cond else '  [FAIL] ') + name
          + ('' if cond else f': {detail}'))
    if not cond:
        FAILS.append(name)


def fp(ref, x, y, side, net, name):
    return (f'  (footprint "T:{ref}" (layer "{side}") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") '
            f'(hide yes) (effects (font (size 1 1) (thickness 0.15))))\n'
            f'    (attr smd)\n    (pad "1" smd rect (at 0 0) (size 1 1) '
            f'(layers "{side}") (net {net} "{name}")))\n')


def build(d, dru=None, pro_sev=None, other_y=110.75, via_size=0.6, ring_min=None):
    """The board, its project (`pro_sev` = rule_severities) and its rules;
    `other_y` is where /OTHER's long track runs (0.5 mm from /SIG's edge at
    the default)."""
    os.makedirs(d, exist_ok=True)
    pcb = os.path.join(d, 'b.kicad_pcb')
    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
         ' (setup (pad_to_mask_clearance 0))\n'
         ' (net 0 "") (net 1 "/SIG") (net 2 "/OTHER")\n')
    b += (fp('A1', 110, 110, 'F.Cu', 1, '/SIG') + fp('A2', 130, 110, 'B.Cu', 1, '/SIG')
          + fp('B1', 113, 112, 'F.Cu', 2, '/OTHER') + fp('B2', 127, 112, 'F.Cu', 2, '/OTHER'))
    ids = iter(range(10, 99))

    def tr(x1, y1, x2, y2, ly, n):
        return (f'  (segment (start {x1} {y1}) (end {x2} {y2}) (width 0.25) '
                f'(layer "{ly}") (net {n}) '
                f'(uuid "00000000-0000-0000-0000-0000000000{next(ids)}"))\n')
    b += tr(110, 110, 120, 110, 'F.Cu', 1) + tr(120, 110, 130, 110, 'B.Cu', 1)
    b += (tr(113, 112, 113, other_y, 'F.Cu', 2) + tr(113, other_y, 127, other_y, 'F.Cu', 2)
          + tr(127, other_y, 127, 112, 'F.Cu', 2))
    b += (f'  (via (at 120 110) (size {via_size}) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1) '
          '(uuid "00000000-0000-0000-0000-000000000001"))\n')
    for x1, y1, x2, y2 in ((100, 100, 140, 100), (140, 100, 140, 120),
                           (140, 120, 100, 120), (100, 120, 100, 100)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(b + ')\n')

    def cl(n):
        return dict(name=n, clearance=0.2, track_width=0.25, via_diameter=0.6,
                    via_drill=0.3, wire_width=6, bus_width=12, line_style=0,
                    priority=(2147483647 if n == 'Default' else 0))
    with open(pcb[:-len('.kicad_pcb')] + '.kicad_pro', 'w', encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {
                       'rules': dict({'min_clearance': 0.1},
                                     **({'min_via_annular_width': ring_min}
                                        if ring_min else {})),
                       'rule_severities': dict(pro_sev or {})}},
                   'meta': {'filename': 'b.kicad_pro', 'version': 3},
                   'net_settings': {'classes': [cl('Default'), cl('HS')],
                                    'meta': {'version': 4},
                                    'netclass_patterns': [{'netclass': 'HS',
                                                           'pattern': '/SIG'}]}},
                  f, indent=1)
    if dru:
        with open(pcb[:-len('.kicad_pcb')] + '.kicad_dru', 'w', encoding='utf-8') as f:
            f.write(f'(version 1)\n{dru}\n')
    return pcb


V = "(constraint disallow via) (condition \"A.NetClass == 'HS'\")"
C = "(constraint clearance (min 0.7mm)) (condition \"A.NetClass == 'HS'\")"


def grade(pcb):
    """(exit code, the --json document)."""
    import subprocess
    js = pcb + '.drc.json'
    # exit 1 is a counted violation here, not a refusal, so the child is run
    # directly and screened for an accident (a traceback, an ImportError)
    r = subprocess.run(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), pcb,
                             '--json', js], capture_output=True, text=True,
                       encoding='utf-8', errors='replace', cwd=REPO, timeout=600)
    acc = run_utils._accident((r.stdout or '') + (r.stderr or ''))
    if acc:
        raise AssertionError(f'BROKEN TEST: check_drc died from {acc!r}\n'
                             + (r.stdout or '')[-800:] + (r.stderr or '')[-800:])
    with open(run_utils.evidence(js, 'check_drc --json'), encoding='utf-8') as f:
        return r.returncode, json.load(f), r.stdout


def run_arm(name, pcb, code, counted, warn, ign, src):
    rc, doc, out = grade(pcb)
    ok(f'{name}: exit {code}', rc == code, rc)
    ok(f'{name}: counted {counted}, warnings {warn}, ignored {ign}',
       (doc['violations'], doc.get('severity_warnings'),
        doc.get('severity_ignored')) == (counted, warn, ign),
       (doc['violations'], doc.get('severity_warnings'),
        doc.get('severity_ignored')))
    demoted = [i for i in doc['items']
               if i.get('accepted') in ('severity-warning', 'severity-ignore')]
    if src:
        ok(f'{name}: every demoted row names {src}',
           demoted and all(i.get('severity_source') == src for i in demoted),
           [i.get('severity_source') for i in demoted])
    if warn:
        ok(f'{name}: listed under WARNINGS, the count line board_score reads',
           f'SEVERITY WARNING: {warn} finding' in out, out[-600:])
    if ign:
        ok(f'{name}: the ignored count is printed',
           f'SEVERITY IGNORE: {ign} finding' in out, out[-600:])


def main():
    tmp = tempfile.mkdtemp(prefix='t1626_')
    try:
        # (name, dru, project severities, expected exit, counted, warnings,
        #  ignored, the source the demoted rows name)
        arms = [
            ('unset', f'(rule no_via {V})', None, 1, 1, 0, 0, None),
            ('rule_warning', f'(rule no_via (severity warning) {V})', None,
             0, 0, 1, 0, "rule 'no_via'"),
            ('clr_rule_warning', f'(rule hs_far (severity warning) {C})', None,
             0, 0, 3, 0, "rule 'hs_far'"),
            ('project_warning', f'(rule no_via {V})', {'items_not_allowed': 'warning'},
             0, 0, 1, 0, 'project rule_severities.items_not_allowed'),
            ('project_ignore', f'(rule hs_far {C})', {'clearance': 'ignore'},
             0, 0, 0, 3, 'project rule_severities.clearance'),
            # KiCad's precedence, measured: the rule's own severity beats the
            # project's warning, and the project's ignore beats the rule
            ('rule_error_project_warning', f'(rule hs_far (severity error) {C})',
             {'clearance': 'warning'}, 1, 3, 0, 0, None),
            ('rule_warning_project_ignore', f'(rule no_via (severity warning) {V})',
             {'items_not_allowed': 'ignore'}, 0, 0, 0, 1,
             'project rule_severities.items_not_allowed'),
        ]
        for name, dru, sev, code, counted, warn, ign, src in arms:
            pcb = build(os.path.join(tmp, name), dru, sev)
            run_arm(name, pcb, code, counted, warn, ign, src)
        # A severity governs KiCad's rule, not the fab floor: copper 0.05 mm
        # apart (below the 0.1 physical clearance floor) on a project that
        # demotes clearance to a warning still counts
        pcb = build(os.path.join(tmp, 'below_fab'), None, {'clearance': 'warning'},
                    other_y=110.3)
        rc, doc, _out = grade(pcb)
        ok('below_fab: a gap under the fab floor keeps counting under a '
           'project warning', rc == 1 and doc['violations'] >= 1
           and not doc.get('severity_warnings'),
           (rc, doc['violations'], doc.get('severity_warnings')))
        pcb = build(os.path.join(tmp, 'above_fab'), None, {'clearance': 'warning'},
                    other_y=110.55)
        rc, doc, _out = grade(pcb)
        ok('above_fab: control, the via 0.125 mm from /OTHER (over the fab '
           'floor, under the 0.2 class) is demoted', rc == 0 and doc['violations'] == 0
           and doc.get('severity_warnings'),
           (rc, doc['violations'], doc.get('severity_warnings')))

        # ... and a size row: a via ring under the board's 0.1 minimum is
        # dropped where the project ignores annular_width, unless it is under
        # the fab floor's ring too ((0.25 - 0.15) / 2 = 0.05 on two layers)
        pcb = build(os.path.join(tmp, 'ring_above_fab'), None,
                    {'annular_width': 'ignore'}, via_size=0.45, ring_min=0.1)
        rc, doc, _out = grade(pcb)
        ok('ring_above_fab: a 0.075 ring under the board minimum is ignored '
           'with the check', rc == 0 and doc.get('severity_ignored') == 1,
           (rc, doc['violations'], doc.get('severity_ignored')))
        pcb = build(os.path.join(tmp, 'ring_below_fab'), None,
                    {'annular_width': 'ignore'}, via_size=0.35, ring_min=0.1)
        rc, doc, _out = grade(pcb)
        ok('ring_below_fab: a 0.025 ring under the fab floor keeps counting',
           rc == 1 and not doc.get('severity_ignored')
           and doc['by_type'].get('via-annular') == 1,
           (rc, doc['by_type'], doc.get('severity_ignored')))

        # board_score: the demoted row is not blocking, and is disclosed
        pcb = os.path.join(tmp, 'rule_warning', 'b.kicad_pcb')
        sj = pcb + '.score.json'
        r = run_utils.check(PY + [os.path.join(REPO, 'py_tools', 'board_score.py'),
                                  pcb, '--json', sj], accept=True)
        with open(run_utils.evidence(sj, 'board_score --json'), encoding='utf-8') as f:
            s = json.load(f)
        ok('board_score: drc 0 on a rule KiCad reports as a warning',
           s['blocking_by']['drc'] == 0, s['blocking_by'])
        ok('board_score: the warning is carried in components.drc',
           s['components']['drc'].get('severity_warnings') == 1,
           s['components']['drc'])
        ok('board_score: and disclosed on its console',
           'KICAD SEVERITY (disclosed, not blocking): 1 ' in r.stdout, r.stdout[-800:])

        # severity_overrides: the staged-at-error grade counts it again
        from check_drc import run_drc
        pcb = os.path.join(tmp, 'project_warning', 'b.kicad_pcb')
        got = run_drc(pcb, quiet=True, print_summary=False)
        ok('in-process: the project-warning row comes back accepted',
           [v.get('accepted') for v in got if v['type'] == 'items-not-allowed']
           == ['severity-warning'], [(v['type'], v.get('accepted')) for v in got])
        got = run_drc(pcb, quiet=True, print_summary=False,
                      severity_overrides={'items_not_allowed': 'error'})
        ok('severity_overrides: staged at error, it is counted',
           [v.get('accepted') for v in got if v['type'] == 'items-not-allowed']
           == [None], [(v['type'], v.get('accepted')) for v in got])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f'FAILED: {len(FAILS)}: {FAILS}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
