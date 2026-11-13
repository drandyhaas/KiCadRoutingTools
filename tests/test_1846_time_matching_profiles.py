#!/usr/bin/env python3
"""#1846: `--time-matching` times a net with its class's tuning profile.

The router's time matching computed delay from the stackup dielectric alone,
while KiCad 10's DRC times a net with the per-layer delay of the tuning
profile its net class names. On the fixture (tests/fixtures/1846: nets `P`
20 mm and `Q` 21 mm on F.Cu, class `TIMED` with profile `PROF6` at 6.0 ps/mm,
a stackup that gives 5.53 ps/mm) the router tuned to 110.63 / 116.16 ps where
KiCad reads 120.00 / 126.00.

Checks:
  1. `impedance.calculate_route_propagation_time_ps` gives KiCad's delays
     for copper of a profiled net (track and via, as
     tests/test_1846_time_domain_rules.py measured them), and the stackup's
     for a net with no profile;
  2. `get_layer_ps_per_mm(pcb, layer, net_id)` is the profile's delay for
     that net (0 for a layer it has no entry for), the stackup's without one;
  3. `route.py --time-matching` on the fixture with its two tracks removed:
     the group's target is 126.00 ps and P is given 6.00 ps, and the routed
     board meets the fixture's own ps rule in check_drc.
"""
import contextlib
import copy
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
from run_utils import check, evidence  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
import impedance  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(TESTS, 'fixtures', '1846', 'time_rule')
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def near(a, b, tol=0.01):
    return abs(a - b) <= tol


def _seg(x1, x2, layer):
    return (f'\t(segment\n\t\t(start {x1} 6)\n\t\t(end {x2} 6)\n\t\t(width 0.1)\n'
            f'\t\t(layer "{layer}")\n\t\t(net "Q")\n\t)\n')


def _via(x):
    return (f'\t(via\n\t\t(at {x} 6)\n\t\t(size 0.46)\n\t\t(drill 0.2)\n'
            f'\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "Q")\n\t)\n')


VIAS = _seg(3, 10, 'F.Cu') + _via(10) + _seg(10, 17, 'B.Cu') + _via(17) + _seg(17, 24, 'F.Cu')


def stage(td, name, vias=False, edit=None, strip=False):
    with open(evidence(BOARD + '.kicad_pcb'), encoding='utf-8') as fh:
        t = fh.read()
    if vias:
        t, n = re.subn(r'\t\(segment\s*\(start 3 6\).*?\n\t\)\n', lambda m: VIAS, t, flags=re.S)
        assert n == 1
    if strip:
        t, n = re.subn(r'\t\(segment\s*\(start 3 [36]\).*?\n\t\)\n', '', t, flags=re.S)
        assert n == 2, 'the fixture has two tracks'
    b = os.path.join(td, name + '.kicad_pcb')
    with open(b, 'w', encoding='utf-8') as fh:
        fh.write(t)
    with open(evidence(BOARD + '.kicad_pro'), encoding='utf-8') as fh:
        proj = copy.deepcopy(json.load(fh))
    if edit:
        edit(proj)
    with open(os.path.join(td, name + '.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(proj, fh, indent=2)
    with open(evidence(BOARD + '.kicad_dru'), encoding='utf-8') as fh:
        dru = fh.read()
    with open(os.path.join(td, name + '.kicad_dru'), 'w', encoding='utf-8') as fh:
        fh.write(dru)
    return evidence(b, 'the staged board')


def load(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def copper(pcb, name):
    nid = next(i for i, n in pcb.nets.items() if n.name == name)
    return nid, [s for s in pcb.segments if s.net_id == nid], [v for v in pcb.vias
                                                                  if v.net_id == nid]


def _bcu(proj):
    proj['tuning_profiles']['tuning_profiles_impedance_geometric'][0]['layer_entries'].append(
        {"signal_layer": "B.Cu", "top_reference_layer": "In2.Cu",
         "bottom_reference_layer": "", "delay": 7000000, "diff_pair_gap": 0, "width": 100000})


def _no_profile(proj):
    for c in proj['net_settings']['classes']:
        c.pop('tuning_profile', None)


def delays(td):
    print("1-2. a profiled net is timed with its profile")
    pcb = load(stage(td, 'a'))
    got = {}
    for name in ('P', 'Q'):
        nid, segs, vias = copper(pcb, name)
        got[name] = impedance.calculate_route_propagation_time_ps(segs, vias, pcb)
    ok(near(got['P'], 120.0) and near(got['Q'], 126.0),
       f"P {got['P']:.2f} ps, Q {got['Q']:.2f} ps (KiCad: 120.00, 126.00)")
    nid = copper(pcb, 'P')[0]
    ok(impedance.get_layer_ps_per_mm(pcb, 'F.Cu', nid) == 6.0
       and near(impedance.get_layer_ps_per_mm(pcb, 'F.Cu'), 5.532, 0.001)
       and impedance.get_layer_ps_per_mm(pcb, 'B.Cu', nid) == 0.0,
       "F.Cu is 6.0 ps/mm for the net, 5.532 from the stackup alone, and B.Cu "
       "(no entry in the profile) 0 for the net")

    pcb = load(stage(td, 'v', vias=True, edit=_bcu))
    nid, segs, vias = copper(pcb, 'Q')
    q = impedance.calculate_route_propagation_time_ps(segs, vias, pcb)
    ok(near(q, 151.48), f"Q through two vias with B.Cu at 7 ps/mm: {q:.2f} ps (KiCad: 151.48)")

    pcb = load(stage(td, 'n', edit=_no_profile))
    nid, segs, vias = copper(pcb, 'P')
    p = impedance.calculate_route_propagation_time_ps(segs, vias, pcb)
    ok(near(p, 110.63) and near(impedance.get_layer_ps_per_mm(pcb, 'F.Cu', nid), 5.532, 0.001),
       f"a net with no profile is timed from the stackup, as before: {p:.2f} ps")


def routed(td):
    print("3. route.py --time-matching")
    board = stage(td, 'in', strip=True)
    out = os.path.join(td, 'out.kicad_pcb')
    r = check(PY + [os.path.join(REPO, 'py_router', 'route.py'), board, out,
                    '--nets', 'P', 'Q', '--length-match-group', 'P', 'Q',
                    '--time-matching', '--time-match-tolerance', '1.0'],
              accept=True, timeout=1800)
    ok('target=126.00ps' in r.stdout and 'P: 120.00ps -> adding 6.00ps' in r.stdout,
       "the group's target is 126.00 ps and P is given 6.00 ps")
    dru = os.path.splitext(out)[0] + '.kicad_dru'
    if not os.path.isfile(dru):
        with open(BOARD + '.kicad_dru', encoding='utf-8') as fh, \
                open(dru, 'w', encoding='utf-8') as oh:
            oh.write(fh.read())
    r = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'),
                    evidence(out, 'the routed board')], accept=True)
    ok('1 group(s)/pair(s) graded, 0 net(s) out of range' in r.stdout
       and '2 net(s) graded, 0 out of range' in r.stdout,
       "the routed board meets the 5 ps skew and the 100-140 ps window in check_drc")


def main():
    with tempfile.TemporaryDirectory() as td:
        delays(td)
        routed(td)
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
