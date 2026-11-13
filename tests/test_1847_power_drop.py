#!/usr/bin/env python3
"""#1847: voltage drop per load and current per via of a routed power net,
with every load drawing at once.

`plane_resistance.analyze_power_trace_net` names a net's narrowest segment
and its rating; it does not know the loads. `power_drop.solve_net` solves the
net's tracks, vias and copper pours as one resistor network, and
`py_tools/check_power_drop.py` reports it.

The fixture (tests/fixtures/1847) is synthetic, 2 layers, no stackup (1 oz,
1.6 mm): `+3V3` from `J1` through 12 mm of 0.15 mm F.Cu, a via, 7 mm of
0.15 mm B.Cu, a via, and two 3.61 mm branches of 0.10 mm F.Cu to `U1`
(0.5 A) and `U2` (0.3 A).

Checks (no routing; every expected value is worked here from rho, length,
width and thickness, not read back from the solver):
  1. the fixture: the drop at each load, 0.8 A in each via and trunk track,
     0.5 / 0.3 A in the branches, and the items over their IPC rating;
  2. two vias in parallel share the current, a track that ends on the middle
     of another joins it there, and a second source pad lowers the drop;
     copper joins wherever it overlaps: a track end short of a pad by less
     than its half-width, a track passing over a pad, two track ends that
     overlap without meeting, a tee off the trunk's centreline;
  3. a copper pour is solved as a sheet: a strip of pour between two pads
     that span it reads rho * L / (W * t), and a pour in parallel with a
     track lowers the drop;
  4. what is not solved is said: a load with no path to a source, a pad
     reference that is not on the net, thermal spokes;
  5. the command prints the report, writes it as JSON, exits 0, and refuses
     a missing or malformed loads file by name.
"""
import contextlib
import io
import json
import math
import os
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
import power_drop  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
TOOL = os.path.join(REPO, 'py_tools', 'check_power_drop.py')
BOARD = os.path.join(TESTS, 'fixtures', '1847', 'power_drop')
LOADS = {'U1.1': 0.5, 'U2.1': 0.3}

RHO = power_drop.COPPER_RESISTIVITY
T_CU = 35e-6                       # 1 oz
WALL = power_drop.DEFAULT_VIA_PLATING_MM


def r_track(length_mm, width_mm):
    return RHO * length_mm * 1e-3 / (width_mm * 1e-3 * T_CU)


def r_via(drill_mm=0.2, height_mm=1.6):
    return RHO * height_mm * 1e-3 / (math.pi * (drill_mm - WALL) * WALL * 1e-6)


BRANCH = math.hypot(3, 2)
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def near(a, b, tol=1e-3):
    return a is not None and abs(a - b) <= tol


def _seg(x1, y1, x2, y2, layer='F.Cu', w=0.15, net='+3V3'):
    return (f'\t(segment\n\t\t(start {x1} {y1})\n\t\t(end {x2} {y2})\n\t\t(width {w})\n'
            f'\t\t(layer "{layer}")\n\t\t(net "{net}")\n\t)\n')


def _via(x, y, net='+3V3'):
    return (f'\t(via\n\t\t(at {x} {y})\n\t\t(size 0.46)\n\t\t(drill 0.2)\n'
            f'\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "{net}")\n\t)\n')


def load(td, name, add='', drop=()):
    """The fixture, with ``add`` appended and each (x1 y1) track start in
    ``drop`` -- as (start, end) texts -- removed, parsed."""
    import re
    with open(evidence(BOARD + '.kicad_pcb'), encoding='utf-8') as fh:
        t = fh.read()
    for start, end in drop:
        t, n = re.subn(r'\t\(segment\s*\(start %s\)\s*\(end %s\).*?\n\t\)\n'
                       % (re.escape(start), re.escape(end)), '', t, flags=re.S)
        assert n == 1, f'one track runs {start} to {end}'
    if add:
        k = t.rindex('(embedded_fonts')
        k = t.rindex('\n', 0, k) + 1
        t = t[:k] + add + t[k:]
    path = os.path.join(td, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(t)
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    return pcb, next(i for i, n in pcb.nets.items() if n.name == '+3V3'), path


def drops(res):
    return {r['pad']: r.get('drop_mv') for r in res['loads']}


def fixture(td):
    print("1. the issue's board")
    pcb, nid, _ = load(td, 'f')
    res = power_drop.solve_net(pcb, nid, ['J1.1'], LOADS)
    trunk = r_track(12, 0.15) + r_track(7, 0.15) + 2 * r_via()
    want = {'U1.1': (0.8 * trunk + 0.5 * r_track(BRANCH, 0.1)) * 1e3,
            'U2.1': (0.8 * trunk + 0.3 * r_track(BRANCH, 0.1)) * 1e3}
    got = drops(res)
    ok(all(near(got[k], want[k]) for k in want),
       f"drops {got} mV, by hand U1 {want['U1.1']:.4f} U2 {want['U2.1']:.4f}")
    ok(near(res['max_drop_mv'], want['U1.1']) and res['total_current_a'] == 0.8,
       "the worst drop is U1's, and 0.8 A is solved")
    ok(sorted(round(v['current_a'], 6) for v in res['vias']) == [0.8, 0.8],
       "each via carries the 0.8 A of both loads")
    amps = sorted((s['width_mm'], round(s['current_a'], 6)) for s in res['segments'])
    ok(amps == [(0.1, 0.3), (0.1, 0.5), (0.15, 0.8), (0.15, 0.8)],
       f"track currents by width: {amps}")
    over = sorted((s['width_mm'], round(s['current_a'], 3)) for s in res['segments']
                  if s['over'])
    ok(over == [(0.1, 0.5), (0.15, 0.8), (0.15, 0.8)]
       and all(v['over'] for v in res['vias']) and res['over'] == 5,
       f"over their rating: tracks {over} and both vias; the 0.3 A branch is not")
    ok(not res['not_modelled'] and not res['errors'], "nothing is left out of this solve")


def networks(td):
    print("2. parallel vias, a tee, a second source")
    pcb, nid, _ = load(td, 'p', _via(15, 6.6) + _seg(15, 6, 15, 6.6)
                       + _seg(15, 6, 15, 6.6, 'B.Cu'))
    res = power_drop.solve_net(pcb, nid, ['J1.1'], LOADS)
    pair = sorted(round(v['current_a'], 3) for v in res['vias'] if v['at'][0] < 16)
    # the second via sits off the trunk, behind 0.6 mm of track on each side
    stub = 2 * r_track(0.6, 0.15)
    share = 0.8 * r_via() / (2 * r_via() + stub)
    ok(near(pair[0], share, 2e-3) and near(sum(pair), 0.8, 2e-3),
       f"two vias in parallel share 0.8 A: {pair} (by hand {share:.3f} / {0.8 - share:.3f})")

    # a spur from the MIDDLE of the F.Cu trunk (x=9) down to a load pad
    pad = ('\t(footprint "L"\n\t\t(layer "F.Cu")\n\t\t(at 9 9)\n'
           '\t\t(property "Reference" "U3"\n\t\t\t(at 0 0 0)\n\t\t\t(layer "F.SilkS")\n\t\t)\n'
           '\t\t(pad "1" smd circle\n\t\t\t(at 0 0)\n\t\t\t(size 0.4 0.4)\n'
           '\t\t\t(layers "F.Cu")\n\t\t\t(net "+3V3")\n\t\t)\n\t)\n')
    pcb, nid, _ = load(td, 't', pad + _seg(9, 6, 9, 9))
    res = power_drop.solve_net(pcb, nid, ['J1.1'], {'U3.1': 0.2})
    want = 0.2 * (r_track(6, 0.15) + r_track(3, 0.15)) * 1e3
    ok(near(drops(res)['U3.1'], want),
       f"a track ending on the middle of another joins it: {drops(res)['U3.1']} mV "
       f"(by hand {want:.4f})")

    pcb, nid, _ = load(td, 's')
    one = power_drop.solve_net(pcb, nid, ['J1.1'], LOADS)
    two = power_drop.solve_net(pcb, nid, ['J1.1', 'U2.1'], {'U1.1': 0.5})
    ok(near(drops(two)['U1.1'],
            0.5 * 1e3 / (1 / (r_track(12, 0.15) + r_track(7, 0.15) + 2 * r_via())
                         + 1 / r_track(BRANCH, 0.1)) + 0.5 * r_track(BRANCH, 0.1) * 1e3)
       and drops(two)['U1.1'] < drops(one)['U1.1'],
       f"a second source in parallel lowers the drop: {drops(two)['U1.1']} mV")


def overlaps(td):
    print("2b. copper joins where it overlaps, as KiCad connects it")
    trunk = r_track(12, 0.15) + r_track(7, 0.15) + 2 * r_via()
    # U1's branch ends 0.04 mm short of its 0.4 mm pad: the 0.1 mm track's
    # copper still overlaps the pad's
    pcb, nid, _ = load(td, 'o1', _seg(22, 6, 25, 4.24, w=0.1), drop=(('22 6', '25 4'),))
    res = power_drop.solve_net(pcb, nid, ['J1.1'], LOADS)
    want = (0.8 * trunk + 0.5 * r_track(math.hypot(3, 1.76), 0.1)) * 1e3
    ok(near(drops(res)['U1.1'], want),
       f"a track ending within its half-width of a pad joins it: "
       f"{drops(res)['U1.1']} mV (by hand {want:.4f})")
    # U1's branch runs on past the pad's centre, with no end on the pad
    pcb, nid, _ = load(td, 'o2', _seg(22, 6, 26.5, 3, w=0.1), drop=(('22 6', '25 4'),))
    res = power_drop.solve_net(pcb, nid, ['J1.1'], LOADS)
    want = (0.8 * trunk + 0.5 * r_track(BRANCH, 0.1)) * 1e3
    ok(near(drops(res)['U1.1'], want),
       f"a track passing over a pad is cut there and joins it: "
       f"{drops(res)['U1.1']} mV (by hand {want:.4f})")
    # the B.Cu trunk as two tracks whose ends overlap 0.05 mm without meeting
    pcb, nid, _ = load(td, 'o3', _seg(15, 6, 18.5, 6, 'B.Cu') + _seg(18.6, 6, 22, 6, 'B.Cu'),
                       drop=(('15 6', '22 6'),))
    res = power_drop.solve_net(pcb, nid, ['J1.1'], LOADS)
    want = (0.8 * (trunk - r_track(0.1, 0.15)) + 0.5 * r_track(BRANCH, 0.1)) * 1e3
    ok(near(drops(res)['U1.1'], want),
       f"two track ends whose copper overlaps join: {drops(res)['U1.1']} mV "
       f"(by hand {want:.4f})")
    # a spur ending 0.06 mm off the F.Cu trunk's centreline, inside its copper
    pad = ('\t(footprint "L"\n\t\t(layer "F.Cu")\n\t\t(at 9 9)\n'
           '\t\t(property "Reference" "U3"\n\t\t\t(at 0 0 0)\n\t\t\t(layer "F.SilkS")\n\t\t)\n'
           '\t\t(pad "1" smd circle\n\t\t\t(at 0 0)\n\t\t\t(size 0.4 0.4)\n'
           '\t\t\t(layers "F.Cu")\n\t\t\t(net "+3V3")\n\t\t)\n\t)\n')
    pcb, nid, _ = load(td, 'o4', pad + _seg(9, 6.06, 9, 9))
    res = power_drop.solve_net(pcb, nid, ['J1.1'], {'U3.1': 0.2})
    want = 0.2 * (r_track(6, 0.15) + r_track(2.94, 0.15)) * 1e3
    ok(near(drops(res)['U3.1'], want),
       f"a tee ending within the trunk's half-width joins it: {drops(res)['U3.1']} mV "
       f"(by hand {want:.4f})")


def _pad_b(ref, x, y):
    return (f'\t(footprint "P"\n\t\t(layer "F.Cu")\n\t\t(at {x} {y})\n'
            f'\t\t(property "Reference" "{ref}"\n\t\t\t(at 0 0 0)\n'
            f'\t\t\t(layer "F.SilkS")\n\t\t)\n'
            f'\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 0.5 2)\n'
            f'\t\t\t(layers "B.Cu")\n\t\t\t(net "+3V3")\n\t\t)\n\t)\n')


def _zone(x0, y0, x1, y1, layer='B.Cu'):
    return (f'\t(zone\n\t\t(net "+3V3")\n\t\t(net_name "+3V3")\n\t\t(layer "{layer}")\n'
            '\t\t(hatch edge 0.5)\n\t\t(connect_pads yes\n\t\t\t(clearance 0.2)\n\t\t)\n'
            '\t\t(min_thickness 0.25)\n\t\t(fill\n\t\t\t(thermal_gap 0.5)\n'
            '\t\t\t(thermal_bridge_width 0.5)\n\t\t)\n\t\t(polygon\n\t\t\t(pts\n'
            f'\t\t\t\t(xy {x0} {y0}) (xy {x1} {y0}) (xy {x1} {y1}) (xy {x0} {y1})\n'
            '\t\t\t)\n\t\t)\n\t)\n')


def pours(td):
    print("3. a copper pour is solved as a sheet")
    # a 20 x 2 mm strip of B.Cu pour, a 0.5 x 2 mm pad across each end
    pcb, nid, _ = load(td, 'z', _zone(5, 8, 25, 10) + _pad_b('P1', 5.25, 9)
                       + _pad_b('P2', 24.75, 9))
    res = power_drop.solve_net(pcb, nid, ['P1.1'], {'P2.1': 1.0})
    sheet = RHO / T_CU
    want = sheet * 19.0 / 2.0 * 1e3                 # 19 mm between the pads' edges
    got = drops(res)['P2.1']
    ok(got is not None and abs(got - want) / want < 0.02,
       f"a strip between two pads: {got} mV at 1 A, rho*L/(W*t) = {want:.4f} mV")
    ok(len(res['pours']) == 1 and res['pours'][0]['layer'] == 'B.Cu'
       and near(res['pours'][0]['sheet_mohm_per_sq'], sheet * 1e3)
       and any('thermal-relief' in n for n in res['not_modelled']),
       f"the pour is reported solved, the spokes named as not modelled: {res['pours']}")
    fine = power_drop.solve_net(pcb, nid, ['P1.1'], {'P2.1': 1.0}, pour_mesh_mm=0.125)
    ok(abs(drops(fine)['P2.1'] - want) / want < 0.02
       and fine['pours'][0]['cells'] > 2 * res['pours'][0]['cells'],
       f"at half the mesh: {drops(fine)['P2.1']} mV on {fine['pours'][0]['cells']} cells")
    # the fixture's own net with a pour under its B.Cu trunk: the via-to-via
    # run gains a parallel sheet, so each load's drop falls
    pcb0, nid0, _ = load(td, 'z0')
    bare = power_drop.solve_net(pcb0, nid0, ['J1.1'], LOADS)
    pcb, nid, _ = load(td, 'z1', _zone(13, 4, 24, 8))
    poured = power_drop.solve_net(pcb, nid, ['J1.1'], LOADS)
    floor = (0.8 * (r_track(12, 0.15) + 2 * r_via()) + 0.5 * r_track(BRANCH, 0.1)) * 1e3
    ok(floor < drops(poured)['U1.1'] < drops(bare)['U1.1'] - 10,
       f"a pour beside the B.Cu trunk: U1 {drops(bare)['U1.1']} -> "
       f"{drops(poured)['U1.1']} mV (never below {floor:.2f}, the rest of the path)")


def disclosed(td):
    print("4. what is not solved is said")
    pcb, nid, _ = load(td, 'u', drop=(('22 6', '25 4'),))        # U1's branch
    res = power_drop.solve_net(pcb, nid, ['J1.1'], LOADS)
    rows = {r['pad']: r for r in res['loads']}
    cut = [p for p, r in rows.items() if r.get('unreached')]
    ok(cut == ['U1.1'] and 'drop_mv' not in rows['U1.1']
       and any('no copper path' in n for n in res['not_modelled'])
       and res['total_current_a'] == 0.3 and 'drop_mv' in rows['U2.1'],
       f"a load cut off from the source is unreached, with no number: {cut}")
    res = power_drop.solve_net(pcb, nid, ['J9.1'], {'U1.1': 0.5, 'R7.2': 0.1})
    ok(any('J9.1 is not a pad' in e for e in res['errors'])
       and any('R7.2 is not a pad' in e for e in res['errors'])
       and any('no source pad' in e for e in res['errors']),
       f"references that are not pads of the net are errors: {res['errors']}")


def command(td):
    print("5. the command")
    out = os.path.join(td, 'report.json')
    loads = evidence(BOARD + '.loads.json')
    r = check(PY + [TOOL, BOARD + '.kicad_pcb', '--loads', loads, '--json', out],
              accept=True)
    ok('worst drop 60.4 mV' in r.stdout and 'U1.1: 0.5 A, drop 60.4 mV' in r.stdout
       and 'OVER' in r.stdout and '(assumed)' in r.stdout,
       "the report gives the drops, flags what is over, and says the plating is assumed")
    with open(evidence(out, 'the JSON report'), encoding='utf-8') as fh:
        doc = json.load(fh)
    ok(doc['nets'][0]['net'] == '+3V3' and len(doc['nets'][0]['vias']) == 2
       and doc['via_plating_assumed'] is True, "the JSON carries the same solve")
    r = check(PY + [TOOL, BOARD + '.kicad_pcb', '--loads', loads, '--via-plating-um', '18'],
              accept=True)
    ok('18 um plating' in r.stdout and '(assumed)' not in r.stdout
       and 'worst drop 60.4 mV' not in r.stdout,
       "a stated plating is used and no longer called assumed")
    check(PY + [TOOL, BOARD + '.kicad_pcb', '--loads', os.path.join(td, 'none.json')],
          refuse='loads file not found', code=3)
    bad = os.path.join(td, 'bad.json')
    with open(bad, 'w', encoding='utf-8') as fh:
        fh.write('{"nets": {"+3V3": {"sources": "J1.1", "loads": {}}}}')
    check(PY + [TOOL, BOARD + '.kicad_pcb', '--loads', bad],
          refuse='"sources" must be a list', code=2)
    ok(True, "a missing loads file exits 3 and a malformed one exits 2, each by name")
    other = os.path.join(td, 'other.json')
    with open(other, 'w', encoding='utf-8') as fh:
        fh.write('{"nets": {"+9V": {"sources": ["J1.1"], "loads": {"U1.1": 1}}}}')
    r = check(PY + [TOOL, BOARD + '.kicad_pcb', '--loads', other], accept=True)
    ok('+9V: not a net of this board' in r.stdout, "a net the board lacks is named")


def main():
    with tempfile.TemporaryDirectory() as td:
        fixture(td)
        networks(td)
        overlaps(td)
        pours(td)
        disclosed(td)
        command(td)
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
