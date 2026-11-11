#!/usr/bin/env python3
"""#1801: a length-match group member that a phase after the run's matching
pass routed is in its group's record, graded, and named.

route.py matches its groups once, before the own-requirement retry (#1618),
the rescue and the terminal escalation. On the board below the main loop
fails SIG (a .kicad_dru rule holds HV's class at 0.5 mm, which fences SIG out
of its only gap) and the own-requirement retry then routes it. The group
`SIG HV` formed with HV alone, so the summary listed `members: ['HV']`, no
`member_status`, and nothing said why, while the two shipped 1.5 mm apart.

Rows:
  1. precondition: the main loop failed SIG and the own-requirement retry
     recovered it (else this board tests nothing);
  2. SIG is in `members`, every member has a `member_status`, and SIG's
     names the phase that routed it;
  3. the shipped lengths are out of tolerance, so some member is not
     `matched`, carries the late-member reason, and the group is not
     recorded protected;
  4. the console names SIG and the phase;
  5. the GUI leg (batch_route in memory) carries the same record;
  6. control: with no rule both route in the main loop and are matched
     and protected, as before.

    python3 tests/test_1801_late_member_group.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check as run_check, evidence, tool        # noqa: E402
from synth import RoutingBoard                                   # noqa: E402

FAILS = []
REASON = 'member-recovered-after-matching'


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def build(path, rule):
    """SIG's only path is a 0.9 mm gap between two foreign pads; with `rule`
    a .kicad_dru clearance holds HV's class at 0.5 mm."""
    b = RoutingBoard(w=20, h=20)
    b.pad('S1', 3, 5, 'SIG', sx=1, sy=1)
    b.pad('S2', 17, 5, 'SIG', sx=1, sy=1)
    b.pad('W1', 10, 2.425, 'GND', sx=1, sy=4.25)
    b.pad('W2', 10, 11.5, '+3V3', sx=1, sy=12.1)
    b.pad('W3', 10, 19.075, '+5V', sx=1, sy=1.25)
    b.pad('H1', 2, 8, 'HV', sx=1, sy=1)
    b.pad('H2', 8, 18, 'HV', sx=1, sy=1)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(b.text())

    def cl(n):
        return {'name': n, 'clearance': 0.2, 'track_width': 0.25,
                'via_diameter': 0.6, 'via_drill': 0.3,
                'priority': 2147483647 if n == 'Default' else 0}
    base = path[:-len('.kicad_pcb')]
    with open(base + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rules': {
            'min_clearance': 0.2, 'min_track_width': 0.2,
            'min_copper_edge_clearance': 0.2, 'min_hole_clearance': 0.25,
            'min_via_diameter': 0.5}}},
            'meta': {'filename': os.path.basename(base) + '.kicad_pro', 'version': 3},
            'net_settings': {'classes': [cl('Default'), cl('HV')],
                             'meta': {'version': 4},
                             'netclass_patterns': [{'netclass': 'HV',
                                                    'pattern': 'HV*'}]}},
            fh, indent=1)
    if rule:
        with open(base + '.kicad_dru', 'w', encoding='utf-8') as fh:
            fh.write('(version 1)\n(rule "HV"\n  (constraint clearance (min 0.5mm))\n'
                     '  (condition "A.NetClass == \'HV\'"))\n')


def route(inp, out):
    js = out[:-len('.kicad_pcb')] + '.json'
    r = run_check([sys.executable, '-B', '-X', 'utf8', tool('route.py'),
                   evidence(inp, 'board'), out, '--nets', 'SIG', 'HV',
                   '--layers', 'F.Cu', '--length-match-group', 'SIG', 'HV',
                   '--json-out', js], accept=True)
    with open(evidence(js, '--json-out'), encoding='utf-8') as fh:
        doc = json.load(fh)
    with open(evidence(out[:-len('.kicad_pcb')] + '.kicad_pro', 'project'),
              encoding='utf-8') as fh:
        prot = (json.load(fh).get('kicad_routing_tools') or {}).get('protected_nets') or {}
    return r.stdout + r.stderr, doc, prot


def group(groups):
    return next((g for g in groups or [] if g.get('name') == 'SIG HV'), {})


def main():
    tmp = tempfile.mkdtemp(prefix='t1801_')
    try:
        for arm in ('late', 'control'):
            os.makedirs(os.path.join(tmp, arm))
        late_in = os.path.join(tmp, 'late', 'in.kicad_pcb')
        ctl_in = os.path.join(tmp, 'control', 'in.kicad_pcb')
        build(late_in, True)
        build(ctl_in, False)

        print("1. precondition")
        log, doc, prot = route(late_in, os.path.join(tmp, 'late', 'out.kicad_pcb'))
        sig_failed = 'Routing SIG' in log and 'FAILED: Could not find route' in log
        if not (sig_failed and 'own-requirement retry: SIG recovered' in log):
            print("  [BROKEN TEST] the main loop did not fail SIG and the "
                  "own-requirement retry recover it:\n" + log[-1500:])
            return 1
        print("  PASS the main loop failed SIG, the own-requirement retry routed it")

        print("2. the group record")
        g = group(doc.get('length_match_groups'))
        ms = g.get('member_status') or {}
        check("SIG is a member", 'SIG' in (g.get('members') or []), str(g))
        check("every member has a member_status",
              set(g.get('members') or []) <= set(ms) and len(ms) == 2, str(ms))
        check("SIG names the phase that routed it",
              (ms.get('SIG') or {}).get('recovered_by') == 'the own-requirement retry'
              and (g.get('late_members') or {}).get('SIG'), str(ms.get('SIG')))

        print("3. the verdict")
        lens = {n: r['length_mm'] for n, r in ms.items()}
        out_tol = len(lens) == 2 and abs(lens['SIG'] - lens['HV']) > 0.1
        check("the shipped lengths are out of tolerance (as before the fix)",
              out_tol, str(lens))
        off = [n for n, r in ms.items() if r.get('status') != 'matched']
        check("a member is not matched, with the late-member reason",
              off and all(ms[n].get('reason') == REASON for n in off), str(ms))
        check("the group is not recorded protected",
              'SIG' not in prot and 'HV' not in prot, str(prot))

        print("4. the console")
        check("one line names SIG and the phase",
              "SIG (routed by the own-requirement retry) joined it after matching "
              "ran" in log)

        print("5. GUI leg: batch_route in memory")
        from kicad_parser import parse_kicad_pcb
        from route import batch_route
        pcb = parse_kicad_pcb(late_in)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            _ok, _fail, _t, rd = batch_route(
                late_in, '', ['SIG', 'HV'], layers=['F.Cu'], pcb_data=pcb,
                length_match_groups=[['SIG', 'HV']], return_results=True)
        gg = group(rd.get('length_match_groups'))
        check("results_data's group lists SIG with a member_status",
              'SIG' in (gg.get('members') or [])
              and 'SIG' in (gg.get('member_status') or {}), str(gg))

        print("6. control: no rule")
        log, doc, prot = route(ctl_in, os.path.join(tmp, 'control', 'out.kicad_pcb'))
        g = group(doc.get('length_match_groups'))
        ms = g.get('member_status') or {}
        check("both matched in the main pass",
              {n: r.get('status') for n, r in ms.items()}
              == {'HV': 'matched', 'SIG': 'matched'} and 'late_members' not in g,
              str(g))
        check("and recorded protected",
              prot.get('SIG') == 'length-matched' and prot.get('HV') == 'length-matched',
              str(prot))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
