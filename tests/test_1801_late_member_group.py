#!/usr/bin/env python3
"""#1801 / #1828: a length-match group member that a phase after the run's
matching pass routed joins its group's record; where it is safe the group is
matched with it (#1828), and otherwise it is graded and named (#1801).

route.py matches its groups once, before the own-requirement retry (#1618),
the rescue and the terminal escalation. On the board below the main loop
fails SIG (a .kicad_dru rule holds the HV class at 0.5 mm, which fences SIG
out of its only gap) and the own-requirement retry then routes it.

Arms:
  late     -- group `SIG HV`. It formed with HV alone, so the first pass
              matched nothing; with SIG it is matched now (#1828): HV is
              meandered up to SIG and both ship within the tolerance.
  shorter  -- group `SIG HV*`: HV and HV2 were matched by the first pass and
              SIG is shorter than their target, so SIG alone is meandered up
              to it (#1828).
  longer   -- the same group with SIG LONGER than that target: matching it
              would re-meander the members the first pass finished, so it
              stays disclosed with the #1801 reason.
  multi    -- group `SIG HV` with HV a 3-pad net, whose Phase-3 tap sits on
              its first-pass copper: disclosed.
  control  -- no rule: both route in the main loop, matched and protected.

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
TOL = 0.1                    # route.py's default --length-match-tolerance


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def build(path, rule, sig=(3, 17), hv2=False, h5=False):
    """SIG's only path is a 0.9 mm gap between two foreign pads; with `rule`
    a .kicad_dru clearance holds the HV class at 0.5 mm. `sig` places SIG's
    two pads on y=5, `hv2` adds a second HV-class net on a wider board's
    right side (a meander bump keeps a long pad's bounding circle), `h5`
    gives HV a third pad (a multipoint member with a Phase-3 tap)."""
    b = RoutingBoard(w=26 if hv2 else 20, h=20)
    b.pad('S1', sig[0], 5, 'SIG', sx=1, sy=1)
    b.pad('S2', sig[1], 5, 'SIG', sx=1, sy=1)
    b.pad('W1', 10, 2.425, 'GND', sx=1, sy=4.25)
    b.pad('W2', 10, 11.5, '+3V3', sx=1, sy=12.1)
    b.pad('W3', 10, 19.075, '+5V', sx=1, sy=1.25)
    b.pad('H1', 2, 8, 'HV', sx=1, sy=1)
    b.pad('H2', 8, 18, 'HV', sx=1, sy=1)
    if h5:
        b.pad('H5', 2, 17, 'HV', sx=1, sy=1)
    if hv2:
        b.pad('H3', 20, 7.5, 'HV2', sx=1, sy=1)
        b.pad('H4', 20, 19, 'HV2', sx=1, sy=1)
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


def route(inp, out, nets, group):
    js = out[:-len('.kicad_pcb')] + '.json'
    r = run_check([sys.executable, '-B', '-X', 'utf8', tool('route.py'),
                   evidence(inp, 'board'), out, '--nets', *nets,
                   '--layers', 'F.Cu', '--length-match-group', *group,
                   '--json-out', js], accept=True)
    with open(evidence(js, '--json-out'), encoding='utf-8') as fh:
        doc = json.load(fh)
    with open(evidence(out[:-len('.kicad_pcb')] + '.kicad_pro', 'project'),
              encoding='utf-8') as fh:
        prot = (json.load(fh).get('kicad_routing_tools') or {}).get('protected_nets') or {}
    return r.stdout + r.stderr, doc, prot


def shipped_lengths(out, names):
    """Each net's copper length measured from the WRITTEN board."""
    from kicad_parser import parse_kicad_pcb
    from net_queries import calculate_route_length
    pcb = parse_kicad_pcb(evidence(out, 'routed board'))
    ids = {n.name: i for i, n in pcb.nets.items()}
    return {n: round(calculate_route_length(
        [s for s in pcb.segments if s.net_id == ids[n]],
        [v for v in pcb.vias if v.net_id == ids[n]], pcb), 3) for n in names}


def drc_and_connected(out):
    js = out[:-len('.kicad_pcb')] + '.drc.json'
    run_check([sys.executable, '-B', '-X', 'utf8', tool('check_drc.py'), out,
               '--json', js, '--quiet'], accept=True, allow=('violation',))
    with open(evidence(js, 'check_drc --json'), encoding='utf-8') as fh:
        drc = json.load(fh)
    con = run_check([sys.executable, '-B', '-X', 'utf8', tool('check_connected.py'),
                     out], accept=True)
    return drc, con.stdout


def group(groups, name):
    return next((g for g in groups or [] if g.get('name') == name), {})


def recovered(log):
    return ('Routing SIG' in log and 'FAILED: Could not find route' in log
            and 'own-requirement retry: SIG recovered' in log)


def main():
    tmp = tempfile.mkdtemp(prefix='t1801_')
    try:
        arms = {}
        for arm, kw in (('late', dict(rule=True)),
                        ('shorter', dict(rule=True, sig=(5, 16), hv2=True)),
                        ('longer', dict(rule=True, hv2=True)),
                        ('multi', dict(rule=True, h5=True)),
                        ('control', dict(rule=False))):
            os.makedirs(os.path.join(tmp, arm))
            arms[arm] = os.path.join(tmp, arm, 'in.kicad_pcb')
            build(arms[arm], **kw)

        def run(arm, nets=('SIG', 'HV'), grp=('SIG', 'HV')):
            out = os.path.join(tmp, arm, 'out.kicad_pcb')
            return (out,) + route(arms[arm], out, list(nets), list(grp))

        print("1. late: precondition")
        out, log, doc, prot = run('late')
        if not recovered(log):
            print("  [BROKEN TEST] the main loop did not fail SIG and the "
                  "own-requirement retry recover it:\n" + log[-1500:])
            return 1
        print("  PASS the main loop failed SIG, the own-requirement retry routed it")

        print("2. late: the group record")
        g = group(doc.get('length_match_groups'), 'SIG HV')
        ms = g.get('member_status') or {}
        check("SIG is a member", 'SIG' in (g.get('members') or []), str(g))
        check("every member has a member_status",
              set(g.get('members') or []) <= set(ms) and len(ms) == 2, str(ms))
        check("SIG names the phase that routed it",
              (ms.get('SIG') or {}).get('recovered_by') == 'the own-requirement retry'
              and (g.get('late_members') or {}).get('SIG'), str(ms.get('SIG')))

        print("3. late: matched now (#1828)")
        check("the group was matched with its late member",
              g.get('rematched') is True
              and {n: r.get('status') for n, r in ms.items()}
              == {'HV': 'matched', 'SIG': 'matched'}, str(g))
        lens = shipped_lengths(out, ['SIG', 'HV'])
        check("the shipped board carries both within the tolerance",
              abs(lens['SIG'] - lens['HV']) <= TOL + 1e-6, str(lens))
        check("and the group is recorded protected",
              prot.get('SIG') == 'length-matched' and prot.get('HV') == 'length-matched',
              str(prot))
        drc, con = drc_and_connected(out)
        check("the shipped board is DRC clean",
              drc.get('violations') == 0,
              str({k: drc.get(k) for k in ('violations', 'by_type')}))
        check("and every net connected", 'ALL NETS FULLY CONNECTED' in con,
              con[-400:])

        print("4. late: the console")
        check("one line names SIG and the phase, and the matching",
              "SIG (routed by the own-requirement retry) joined it after matching "
              "ran; matching it now (#1828)" in log, log[-1200:])

        print("5. GUI leg: batch_route in memory")
        from kicad_parser import parse_kicad_pcb
        from route import batch_route
        pcb = parse_kicad_pcb(arms['late'])
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            _ok, _fail, _t, rd = batch_route(
                arms['late'], '', ['SIG', 'HV'], layers=['F.Cu'], pcb_data=pcb,
                length_match_groups=[['SIG', 'HV']], return_results=True)
        gg = group(rd.get('length_match_groups'), 'SIG HV')
        gms = gg.get('member_status') or {}
        check("results_data's group is matched with SIG in it",
              gg.get('rematched') is True
              and {n: r.get('status') for n, r in gms.items()}
              == {'HV': 'matched', 'SIG': 'matched'}, str(gg))

        print("6. shorter: only the late member is meandered (#1828)")
        out, log, doc, prot = run('shorter', ('SIG', 'HV', 'HV2'), ('SIG', 'HV*'))
        if not recovered(log):
            print("  [BROKEN TEST] shorter arm: SIG was not recovered late:\n"
                  + log[-1500:])
            return 1
        g = group(doc.get('length_match_groups'), 'SIG HV*')
        ms = g.get('member_status') or {}
        late_part = log.split('matching it now (#1828)', 1)[-1]
        check("the first pass matched HV and HV2 without SIG",
              'Matched nets: [' in log and "'SIG'" not in
              log.split('Matched nets: [', 1)[-1].split(']', 1)[0], log[-800:])
        check("matched now, every member matched",
              g.get('rematched') is True and len(ms) == 3
              and all(r.get('status') == 'matched' for r in ms.values()), str(g))
        check("the late pass lengthened SIG alone",
              'SIG: ' in late_part and 'HV: ' not in late_part.split('Length match group', 1)[0]
              and 'HV2: ' not in late_part.split('Length match group', 1)[0],
              late_part[:600])
        lens = shipped_lengths(out, ['SIG', 'HV', 'HV2'])
        check("the shipped board carries all three within the tolerance",
              max(lens.values()) - min(lens.values()) <= 2 * TOL + 1e-6, str(lens))
        drc, _con = drc_and_connected(out)
        check("the shipped board is DRC clean", drc.get('violations') == 0,
              str({k: drc.get(k) for k in ('violations', 'by_type')}))

        print("7. longer: the late member would raise the target -> disclosed")
        out, log, doc, prot = run('longer', ('SIG', 'HV', 'HV2'), ('SIG', 'HV*'))
        if not recovered(log):
            print("  [BROKEN TEST] longer arm: SIG was not recovered late:\n"
                  + log[-1500:])
            return 1
        g = group(doc.get('length_match_groups'), 'SIG HV*')
        ms = g.get('member_status') or {}
        off = [n for n, r in ms.items() if r.get('status') != 'matched']
        check("not matched again, the members out of tolerance carry the "
              "late-member reason",
              'rematched' not in g and off
              and all(ms[n].get('reason') == REASON for n in off), str(g))
        check("the record says why it was not matched again",
              'SIG is longer than the target' in (g.get('not_rematched') or ''),
              str(g.get('not_rematched')))
        check("SIG is not recorded protected", 'SIG' not in prot, str(prot))
        check("the #1801 line names it and the reason",
              "joined it after matching ran, so it was not matched (#1801; "
              "SIG is longer than the target" in log)

        print("8. multi: a multipoint member with a Phase-3 tap -> disclosed")
        out, log, doc, prot = run('multi')
        if not (recovered(log) and 'Multi-point Phase 3: Routing' in log):
            print("  [BROKEN TEST] multi arm: SIG was not recovered late, or "
                  "HV took no Phase-3 tap:\n" + log[-1500:])
            return 1
        g = group(doc.get('length_match_groups'), 'SIG HV')
        ms = g.get('member_status') or {}
        off = [n for n, r in ms.items() if r.get('status') != 'matched']
        check("not matched again, disclosed with the late-member reason",
              'rematched' not in g and 'SIG' in (g.get('late_members') or {})
              and off and all(ms[n].get('reason') == REASON for n in off), str(g))
        check("because HV is multipoint",
              g.get('not_rematched') == 'HV is not a 2-pad net',
              str(g.get('not_rematched')))

        print("9. control: no rule")
        out, log, doc, prot = run('control')
        g = group(doc.get('length_match_groups'), 'SIG HV')
        ms = g.get('member_status') or {}
        check("both matched in the main pass",
              {n: r.get('status') for n, r in ms.items()}
              == {'HV': 'matched', 'SIG': 'matched'} and 'late_members' not in g
              and 'rematched' not in g, str(g))
        check("and recorded protected",
              prot.get('SIG') == 'length-matched' and prot.get('HV') == 'length-matched',
              str(prot))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
