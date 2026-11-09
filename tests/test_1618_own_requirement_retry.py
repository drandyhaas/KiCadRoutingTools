#!/usr/bin/env python3
"""#1618: a net fenced out of a legal gap by a wider rule of another routed net.

A routing call prices every foreign obstacle at the widest requirement over
its routed nets: the class floor and the class/type-scoped `.kicad_dru`
stamps. The board: net SIG (Default, 0.2 mm) has one path, a 0.9 mm gap
between a GND pad and a +3V3 pad. HV and HV2 are held at 0.5 mm -- by a rule
(`A.NetClass == 'HV'`) or, in the class arm, by their net class. HV routes
freely, far from that gap. HV2's only path is a second 0.9 mm gap that its
0.5 mm forbids. Routed together, the GND/+3V3 pads are stamped at 0.5 mm
for SIG too. So SIG failed: with the rule it shipped unrouted, and with the
class it shipped only after the rescue narrowed it to 0.127 mm.

Now a net the pipeline leaves open gets one whole-net retry with foreign
copper priced at its OWN requirement (net_rescue.own_requirement_retry),
before the rescue narrows anything.

Checks:
  1. Control: SIG alone routes at its width (the gap is legal for it).
  2. Rule arm, CLI: `--nets SIG HV HV2` -- the retry recovers SIG at full
     width, HV2 stays unrouted, and check_drc passes the board with the rule.
  3. Class arm, CLI: the same with HV in a 0.5 mm class and no rule -- SIG
     routes at full width, not narrowed.
  4. Rule arm, GUI leg (batch_route in memory): the change-set carries SIG's
     copper and none of HV2's.
  5. Inert: with HV at the Default clearance and no rule there is no
     candidate config and no retry line; own_requirement_config answers
     None for every net.

    python3 tests/test_1618_own_requirement_retry.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import evidence                                  # noqa: E402
from synth import RoutingBoard                                  # noqa: E402

failures = []
ROUTE = os.path.join(ROOT, 'py_router', 'route.py')
RETRY = 'own-requirement retry: SIG recovered'
WIDTH = 0.25


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def build(path, hv_clearance=0.2, rule=True):
    b = RoutingBoard(w=20, h=20)
    b.pad('S1', 3, 5, 'SIG', sx=1, sy=1)
    b.pad('S2', 17, 5, 'SIG', sx=1, sy=1)
    b.pad('W1', 10, 2.425, 'GND', sx=1, sy=4.25)        # the wall: GND,
    b.pad('W2', 10, 11.5, '+3V3', sx=1, sy=12.1)        # +3V3 and +5V with
    b.pad('W3', 10, 19.075, '+5V', sx=1, sy=1.25)       # two 0.9 mm gaps
    b.pad('H1', 3, 12, 'HV', sx=1, sy=1)
    b.pad('H2', 7, 12, 'HV', sx=1, sy=1)
    b.pad('K1', 3, 18, 'HV2', sx=1, sy=1)
    b.pad('K2', 17, 18, 'HV2', sx=1, sy=1)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(b.text())

    def cl(n, c):
        return {'name': n, 'clearance': c, 'track_width': WIDTH, 'via_diameter': 0.6,
                'via_drill': 0.3, 'priority': 2147483647 if n == 'Default' else 0}
    base = path[:-len('.kicad_pcb')]
    with open(base + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rules': {
            'min_clearance': 0.2, 'min_track_width': 0.2, 'min_copper_edge_clearance': 0.2,
            'min_hole_clearance': 0.25, 'min_via_diameter': 0.5}}},
            'meta': {'filename': os.path.basename(base) + '.kicad_pro', 'version': 3},
            'net_settings': {'classes': [cl('Default', 0.2), cl('HV', hv_clearance)],
                             'meta': {'version': 4},
                             'netclass_patterns': [{'netclass': 'HV', 'pattern': 'HV*'}]}},
            fh, indent=1)
    if rule:
        with open(base + '.kicad_dru', 'w', encoding='utf-8') as fh:
            fh.write('(version 1)\n(rule "HV"\n  (constraint clearance (min 0.5mm))\n'
                     '  (condition "A.NetClass == \'HV\'"))\n')
    evidence(path, 'synthetic input board')


def net_widths(path, name):
    from kicad_parser import parse_kicad_pcb
    p = parse_kicad_pcb(path)
    nid = next(i for i, n in p.nets.items() if n.name == name)
    return [round(s.width, 4) for s in p.segments if s.net_id == nid]


def route(inp, out, nets):
    r = subprocess.run([sys.executable, ROUTE, inp, out, '--nets', *nets,
                        '--layers', 'F.Cu'], capture_output=True, text=True,
                       cwd=ROOT, timeout=900)
    if r.returncode != 0:
        print(f"  [BROKEN TEST] route.py exited {r.returncode}:\n{r.stdout[-1500:]}"
              f"\n{r.stderr[-1500:]}")
        sys.exit(1)
    evidence(out, 'routed output')
    return r.stdout


def drc_clean(path):
    r = subprocess.run([sys.executable, os.path.join(ROOT, 'py_router', 'check_drc.py'),
                        path], capture_output=True, text=True, cwd=ROOT, timeout=600)
    return 'NO DRC VIOLATIONS' in r.stdout


def main():
    tmp = tempfile.mkdtemp(prefix='t1618_')
    try:
        rule = os.path.join(tmp, 'rule', 'in.kicad_pcb')
        cls = os.path.join(tmp, 'cls', 'in.kicad_pcb')
        flat = os.path.join(tmp, 'flat', 'in.kicad_pcb')
        for p in (rule, cls, flat):
            os.makedirs(os.path.dirname(p))
        build(rule)
        build(cls, hv_clearance=0.5, rule=False)
        build(flat, rule=False)

        print("1. control: SIG alone")
        out = os.path.join(tmp, 'rule', 'sig.kicad_pcb')
        route(rule, out, ['SIG'])
        if net_widths(out, 'SIG') != [WIDTH]:
            print(f"  [BROKEN TEST] SIG alone routed {net_widths(out, 'SIG')}: the gap "
                  f"is not legal for it at {WIDTH} mm")
            return 1
        print("  [ok] SIG routes through its gap")

        print("2. rule arm, CLI: SIG HV HV2 in one call")
        out = os.path.join(tmp, 'rule', 'all.kicad_pcb')
        log = route(rule, out, ['SIG', 'HV', 'HV2'])
        check("the own-requirement retry recovers SIG", RETRY in log)
        check("SIG ships at full width", net_widths(out, 'SIG') == [WIDTH],
              str(net_widths(out, 'SIG')))
        check("HV2, whose gap the rule forbids, stays unrouted",
              net_widths(out, 'HV2') == [])
        check("check_drc passes it with the rule", drc_clean(out))

        print("3. class arm, CLI: HV in a 0.5 mm class, no rule")
        out = os.path.join(tmp, 'cls', 'all.kicad_pcb')
        log = route(cls, out, ['SIG', 'HV', 'HV2'])
        check("the own-requirement retry recovers SIG", RETRY in log)
        check("SIG ships at full width, not narrowed", net_widths(out, 'SIG') == [WIDTH],
              str(net_widths(out, 'SIG')))
        check("check_drc passes it at the classes", drc_clean(out))

        print("4. rule arm, GUI leg: batch_route in memory")
        from kicad_parser import parse_kicad_pcb
        from route import batch_route
        pcb = parse_kicad_pcb(rule)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            _ok, _fail, _t, rd = batch_route(rule, '', ['SIG', 'HV', 'HV2'],
                                             layers=['F.Cu'], pcb_data=pcb,
                                             return_results=True)
        check("the own-requirement retry recovers SIG", RETRY in buf.getvalue())
        sid = next(i for i, n in pcb.nets.items() if n.name == 'SIG')
        hid = next(i for i, n in pcb.nets.items() if n.name == 'HV2')
        emitted = [s.net_id for r in rd.get('results', [])
                   for s in (r.get('new_segments') or [])]
        check("the change-set carries SIG's copper", sid in emitted,
              f"{len(emitted)} segment(s) emitted")
        check("and none of HV2's", hid not in emitted)

        print("5. inert: one requirement for every routed net")
        out = os.path.join(tmp, 'flat', 'all.kicad_pcb')
        log = route(flat, out, ['SIG', 'HV', 'HV2'])
        check("no own-requirement retry runs", 'Own-requirement retry' not in log)
        from routing_config import GridRouteConfig
        from net_rescue import own_requirement_config
        cfg = GridRouteConfig()
        cfg.clearance = 0.2
        cfg.set_net_clearances({5: 0.2, 6: 0.2}, [1, 5, 6])
        check("own_requirement_config answers None on equal classes",
              all(own_requirement_config(cfg, n) is None for n in (1, 5, 6)))
        cfg.set_net_clearances({5: 0.5, 6: 0.5}, [1, 5, 6])
        check("...and a config for the narrower net when a class is wider",
              own_requirement_config(cfg, 1) is not None
              and own_requirement_config(cfg, 1).net_clearance_floor == 0.2
              and own_requirement_config(cfg, 5) is None)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
