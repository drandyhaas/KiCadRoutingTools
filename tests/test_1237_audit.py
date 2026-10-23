#!/usr/bin/env python3
"""#1237: audit.py judges a board against a spec, check by check, with reasons.

Synthetic boards whose answers are computed independently:

1. The spec: comment keys are ignored; an unknown key, a missing required key
   or an impedance without a tolerance is exit 2; a missing board is exit 3.
2. Without a spec only the spec-free checks run, and a clean board exits 0.
3. currents over dc_network:
   - one 10 mm x 0.5 mm track of 35 um copper: the drop at 1 A is rho L / (w t);
   - a second, parallel track halves it;
   - a via adds rho L_barrel / (pi drill t_plating) to the path;
   - capacity fails exactly when the current exceeds the IPC-2152-form
     rating, and so does a via;
   - a dict of loads is solved together; a load joined by no track is
     not_judged, never a pass.
4. decoupling.path: the shortest track path (via barrels counted, as
   net_queries measures length) and its via count, against max_distance_mm
   and max_vias; a pin that does not exist is not_judged.
5. pairs on a stackup board: each coupled stretch's impedance equals
   impedance.calculate_impedance_for_layer at its width and gap; the window
   from gap_max_mm leaves a wider stretch out; skew and uncoupled length;
   no stackup is not_judged. pair.reference: the pair over a GND pour passes,
   and with reference_nets naming another net the same pour does not count.
6. length.match and net.min_width; the board's own min_track_width wins when
   it is stricter than the spec.
7. board_score --spec counts the failed checks in blocking; without --spec
   the audit component is ungraded.

    python3 tests/test_1237_audit.py
"""
import json
import math
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

import audit as audit_mod  # noqa: E402
import dc_network  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from plane_resistance import COPPER_RESISTIVITY, calculate_max_current_ipc2152  # noqa: E402

RUN_ALL_TIMEOUT = 300
TOOL = os.path.join(ROOT, 'py_tools', 'audit.py')
FAILS = []
WORK = tempfile.mkdtemp(prefix='t1237_')


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


STACKUP = ('(stackup (layer "F.Cu" (type "copper") (thickness 0.035))'
           ' (layer "dielectric 1" (type "core") (thickness 0.2) (material "FR4")'
           ' (epsilon_r 4.5) (loss_tangent 0.02))'
           ' (layer "B.Cu" (type "copper") (thickness 0.035)))')


def board(body, stackup=True, name='b'):
    text = ('(kicad_pcb (version 20240202) (generator "t")\n'
            ' (general (thickness 0.27))\n'
            ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (39 "F.Mask" user)'
            ' (38 "B.Mask" user) (44 "Edge.Cuts" user))\n'
            ' (setup %s)\n'
            ' (net 0 "") (net 1 "/P") (net 2 "GND") (net 3 "/A") (net 4 "/B") (net 5 "/N")\n'
            ' (gr_rect (start 80 80) (end 140 120) (layer "Edge.Cuts") (width 0.1))\n'
            '%s\n)\n') % (STACKUP if stackup else '', body)
    path = os.path.join(WORK, name + '.kicad_pcb')
    with open(path, 'w') as fh:
        fh.write(text)
    return path


def pad(ref, x, y, net, name, size='1 1', layers='"F.Cu" "F.Mask"', num='1'):
    return ('(footprint "L:P" (layer "F.Cu") (at %g %g) (property "Reference" "%s")'
            ' (pad "%s" smd rect (at 0 0) (size %s) (layers %s) (net %d "%s")))\n'
            % (x, y, ref, num, size, layers, net, name))


def seg(x1, y1, x2, y2, net, w=0.5, layer='F.Cu'):
    return ('(segment (start %g %g) (end %g %g) (width %g) (layer "%s") (net %d))\n'
            % (x1, y1, x2, y2, w, layer, net))


def via(x, y, net, size=0.6, drill=0.3):
    return ('(via (at %g %g) (size %g) (drill %g) (layers "F.Cu" "B.Cu") (net %d))\n'
            % (x, y, size, drill, net))


def spec_file(spec, name='s'):
    path = os.path.join(WORK, name + '.json')
    with open(path, 'w') as fh:
        json.dump(spec, fh)
    return path


def run(board_path, spec=None, extra=()):
    js = os.path.join(WORK, 'r.json')
    args = [sys.executable, TOOL, board_path, '--json', js] + list(extra)
    if spec is not None:
        args += ['--spec', spec_file(spec)]
    r = subprocess.run(args, capture_output=True, text=True)
    doc = json.load(open(js)) if r.returncode in (0, 4) else None
    return r.returncode, doc, r.stdout + r.stderr


def get(doc, cid, subject=None):
    return next((c for c in doc['checks'] if c['id'] == cid
                 and (subject is None or c.get('subject') == subject)), None)


RHO_OHM_MM = COPPER_RESISTIVITY * 1e3
T_CU = 0.035


def main():
    # ---- 1. the spec ----------------------------------------------------------
    plain = board(pad('J1', 100, 100, 3, '/A') + pad('U1', 110, 100, 3, '/A')
                  + seg(100, 100, 110, 100, 3), name='plain')
    for bad, what in (({'pairs': {'X': {'nets': ['/A', '/B']}}, 'nope': 1}, 'an unknown key'),
                      ({'currents': {'/A': {'amps': 1}}}, 'a missing required key'),
                      ({'pairs': {'X': {'nets': ['/A', '/B'], 'impedance_ohm': 90}}},
                       'an impedance without a tolerance')):
        rc, _d, out = run(plain, bad)
        check(f'1. {what} is exit 2', rc == 2, out[-300:])
    rc, _d, out = run(plain, {'_comment': 'x'})
    check('1. a comment key is ignored', rc == 0, out[-300:])
    rc, _d, _o = run(os.path.join(WORK, 'missing.kicad_pcb'))
    check('1. a missing board is exit 3', rc == 3)

    # ---- 2. spec-free ---------------------------------------------------------
    rc, doc, out = run(plain)
    ids = sorted(c['id'] for c in doc['checks'])
    check('2. only the spec-free checks run without a spec',
          ids == sorted(['weird.off_angle_corner', 'weird.short_fragment',
                         'weird.off_centre_via_end', 'via.mask_contact', 'via.mask_web']),
          str(ids))
    check('2. a clean board exits 0', rc == 0, out[-300:])

    # ---- 3. currents ------------------------------------------------------------
    one = RHO_OHM_MM * 10 / (0.5 * T_CU)
    rc, doc, out = run(plain, {'currents': {'/A': {'amps': 1, 'source': 'J1.1',
                                                   'max_drop_mv': 1000}}})
    d = get(doc, 'current.drop', '/A')
    check('3. one track: the drop is rho L / (w t)',
          d and abs(d['measured']['U1.1'] - one * 1e3) < 0.01,
          f"{d and d.get('measured')} vs {one * 1e3:.3f} mV")
    par = board(pad('J1', 100, 100, 3, '/A', size='1 3') + pad('U1', 110, 100, 3, '/A', size='1 3')
                + seg(100, 99, 110, 99, 3) + seg(100, 101, 110, 101, 3), name='par')
    rc, doc, _o = run(par, {'currents': {'/A': {'amps': 1, 'source': 'J1.1',
                                                'max_drop_mv': 1000}}})
    d = get(doc, 'current.drop', '/A')
    check('3. a parallel track halves it',
          d and abs(d['measured']['U1.1'] - one * 1e3 / 2) < 0.01, str(d and d.get('measured')))
    viab = board(pad('J1', 100, 100, 3, '/A') + pad('U1', 110, 100, 3, '/A')
                 + seg(100, 100, 105, 100, 3) + via(105, 100, 3)
                 + seg(105, 100, 105, 100.0001, 3, layer='B.Cu')
                 + seg(105, 100, 110, 100, 3, layer='B.Cu')
                 + via(110, 100, 3), name='viab')
    pcb = parse_kicad_pcb(viab)
    barrel = pcb.get_via_barrel_length('F.Cu', 'B.Cu')
    via_ohm = RHO_OHM_MM * barrel / (math.pi * 0.3 * dc_network.VIA_PLATING_MM)
    rc, doc, _o = run(viab, {'currents': {'/A': {'amps': 1, 'source': 'J1.1',
                                                 'max_drop_mv': 1000}}})
    d = get(doc, 'current.drop', '/A')
    want = (one + 2 * via_ohm) * 1e3
    check('3. a via adds its barrel resistance (two vias here)',
          d and abs(d['measured']['U1.1'] - want) < 0.01, f"{d and d.get('measured')} vs {want:.3f}")
    rating = calculate_max_current_ipc2152(0.5, T_CU / 0.035, 10.0)
    for amps, want_fail in ((rating * 0.9, False), (rating * 1.1, True)):
        rc, doc, _o = run(plain, {'currents': {'/A': {'amps': amps, 'source': 'J1.1'}}})
        c = get(doc, 'current.capacity', '/A')
        check(f'3. capacity at {amps:.2f} A against a {rating:.2f} A rating -> '
              f"{'failed' if want_fail else 'passed'}",
              c['status'] == ('failed' if want_fail else 'passed'), str(c))
    via_rating = calculate_max_current_ipc2152(math.pi * 0.3,
                                               dc_network.VIA_PLATING_MM / 0.035, 10.0)
    rc, doc, _o = run(viab, {'currents': {'/A': {'amps': via_rating * 1.1, 'source': 'J1.1'}}})
    c = get(doc, 'current.via', '/A')
    check('3. a via over its barrel rating fails', c['status'] == 'failed', str(c))
    two = board(pad('J1', 100, 100, 3, '/A') + pad('U1', 110, 100, 3, '/A')
                + pad('U2', 120, 100, 3, '/A') + seg(100, 100, 110, 100, 3)
                + seg(110, 100, 120, 100, 3), name='two')
    rc, doc, _o = run(two, {'currents': {'/A': {'amps': 1, 'source': 'J1.1', 'max_drop_mv': 1000,
                                                'loads': {'U1.1': 0.5, 'U2.1': 0.5}}}})
    d = get(doc, 'current.drop', '/A')['measured']
    check('3. a dict of loads is solved together (both currents through the first leg)',
          abs(d['U1.1'] - one * 1e3) < 0.01 and abs(d['U2.1'] - 1.5 * one * 1e3) < 0.01, str(d))
    cut = board(pad('J1', 100, 100, 3, '/A') + pad('U1', 110, 100, 3, '/A')
                + pad('U2', 130, 100, 3, '/A') + seg(100, 100, 110, 100, 3), name='cut')
    rc, doc, _o = run(cut, {'currents': {'/A': {'amps': 0.1, 'source': 'J1.1', 'max_drop_mv': 1000}}})
    d = get(doc, 'current.drop', '/A')
    check('3. a load joined by no track is not_judged, not passed',
          d['status'] == 'not_judged' and 'U2.1' in d['reason'], str(d))

    # ---- 4. decoupling --------------------------------------------------------
    dec = board(pad('C1', 100, 100, 3, '/A') + pad('U1', 104, 100, 3, '/A')
                + seg(100, 100, 102, 100, 3) + via(102, 100, 3)
                + seg(102, 100, 104, 100, 3, layer='B.Cu') + via(104, 100, 3), name='dec')
    path = 4.0 + 2 * parse_kicad_pcb(dec).get_via_barrel_length('F.Cu', 'B.Cu')
    for limit, vias, want in ((5.0, 2, 'passed'), (3.0, 2, 'failed'), (5.0, 1, 'failed')):
        rc, doc, _o = run(dec, {'decoupling': {'C1': {'ic': 'U1', 'pins': ['1'],
                                                      'max_distance_mm': limit,
                                                      'max_vias': vias}}})
        c = get(doc, 'decoupling.path', 'C1')
        check(f'4. {path:.2f} mm and 2 vias against {limit} mm / {vias} vias -> {want}',
              c['status'] == want and c['measured']['U1.1']['vias'] == 2
              and abs(c['measured']['U1.1']['path_mm'] - path) < 0.001, str(c))
    rc, doc, _o = run(dec, {'decoupling': {'C1': {'ic': 'U1', 'pins': ['9'],
                                                  'max_distance_mm': 5}}})
    check('4. a pin that does not exist is not_judged',
          get(doc, 'decoupling.path', 'C1')['status'] == 'not_judged')

    # ---- 5. pairs -------------------------------------------------------------
    from impedance import calculate_impedance_for_layer
    pair = (pad('J1', 100, 100, 1, '/P', size='0.2 0.2') + pad('J2', 100, 100.35, 5, '/N', size='0.2 0.2')
            + pad('U1', 120, 100, 1, '/P', size='0.2 0.2') + pad('U2', 120, 100.35, 5, '/N', size='0.2 0.2')
            + seg(100, 100, 120, 100, 1, w=0.2) + seg(100, 100.35, 120, 100.35, 5, w=0.2))
    gnd = ('(zone (net 2) (net_name "GND") (layer "B.Cu") (hatch edge 0.5)'
           ' (connect_pads (clearance 0.2)) (min_thickness 0.1) (fill yes)'
           ' (polygon (pts (xy 90 90) (xy 130 90) (xy 130 110) (xy 90 110)))'
           ' (filled_polygon (layer "B.Cu") (pts (xy 90 90) (xy 130 90) (xy 130 110) (xy 90 110))))\n')
    pb = board(pair + gnd, name='pair')
    z = calculate_impedance_for_layer(parse_kicad_pcb(pb), 'F.Cu', 0.2, spacing=0.15)['zdiff']
    for target, want in ((z, 'passed'), (z * 1.5, 'failed')):
        rc, doc, _o = run(pb, {'pairs': {'D': {'nets': ['/P', '/N'], 'impedance_ohm': target,
                                               'tolerance_pct': 5, 'gap_max_mm': 0.2}}})
        c = get(doc, 'pair.impedance', 'D')
        check(f'5. impedance {z:.1f} ohm against {target:.1f} +-5% -> {want}',
              c['status'] == want, str(c))
    c = get(doc, 'pair.impedance', 'D')
    check('5. the finding measures calculate_impedance_for_layer at the stretch',
          c['findings'][0]['measured'] == round(z, 2), str(c['findings'][0]))
    rc, doc, _o = run(pb, {'pairs': {'D': {'nets': ['/P', '/N'], 'impedance_ohm': z,
                                           'tolerance_pct': 5, 'gap_max_mm': 0.1,
                                           'max_uncoupled_mm': 1.0}}})
    check('5. a gap window below the 0.15 mm gap leaves the stretch out',
          get(doc, 'pair.impedance', 'D')['status'] == 'not_judged')
    check('5. ...and the whole pair runs uncoupled',
          get(doc, 'pair.uncoupled', 'D')['status'] == 'failed')
    rc, doc, _o = run(pb, {'pairs': {'D': {'nets': ['/P', '/N'], 'max_skew_mm': 0.01,
                                           'max_uncoupled_mm': 1.0, 'gap_max_mm': 0.2}},
                           'reference_nets': ['GND']})
    check('5. equal members: skew passes', get(doc, 'pair.skew', 'D')['status'] == 'passed')
    check('5. coupled end to end: uncoupled passes',
          get(doc, 'pair.uncoupled', 'D')['status'] == 'passed', str(get(doc, 'pair.uncoupled', 'D')))
    check('5. over the GND pour: reference passes',
          get(doc, 'pair.reference', 'D')['status'] == 'passed', str(get(doc, 'pair.reference', 'D')))
    rc, doc, _o = run(pb, {'pairs': {'D': {'nets': ['/P', '/N']}}, 'reference_nets': ['/B']})
    check('5. with reference_nets naming another net, the GND pour does not count',
          get(doc, 'pair.reference', 'D')['status'] == 'failed', str(get(doc, 'pair.reference', 'D')))
    nostack = board(pair, stackup=False, name='nostack')
    rc, doc, _o = run(nostack, {'pairs': {'D': {'nets': ['/P', '/N'], 'impedance_ohm': 90,
                                                'tolerance_pct': 10}}})
    check('5. no stackup: impedance is not_judged',
          get(doc, 'pair.impedance', 'D')['status'] == 'not_judged')

    # ---- 6. length and widths -------------------------------------------------
    rc, doc, _o = run(pb, {'length_groups': {'G': {'nets': ['/P', '/N'], 'tolerance_mm': 0.01}}})
    check('6. equal lengths pass', get(doc, 'length.match', 'G')['status'] == 'passed')
    rc, doc, _o = run(plain, {'net_min_widths': {'/A': 0.4}})
    check('6. a 0.5 mm track passes a 0.4 mm spec', get(doc, 'net.min_width', '/A')['status'] == 'passed')
    with open(plain[:-len('.kicad_pcb')] + '.kicad_pro', 'w') as fh:
        json.dump({'board': {'design_settings': {'rules': {'min_track_width': 0.6}}}}, fh)
    rc, doc, _o = run(plain, {'net_min_widths': {'/A': 0.4}})
    c = get(doc, 'net.min_width', '/A')
    check("6. the board's stricter 0.6 mm floor wins, and says so",
          c['status'] == 'failed' and c['findings'][0]['limit'] == 0.6
          and c['findings'][0]['limit_from'].startswith('board:'), str(c))
    os.remove(plain[:-len('.kicad_pcb')] + '.kicad_pro')

    # ---- 7. board_score ---------------------------------------------------------
    bs = os.path.join(ROOT, 'py_tools', 'board_score.py')
    js = os.path.join(WORK, 'score.json')
    sp = spec_file({'net_min_widths': {'/A': 0.6}}, 'bs')
    subprocess.run([sys.executable, bs, plain, '--spec', sp, '--json', js],
                   capture_output=True, text=True)
    score = json.load(open(js))
    check('7. board_score --spec counts the failed spec check',
          score['blocking_by'].get('audit') == 1, str(score['blocking_by']))
    subprocess.run([sys.executable, bs, plain, '--json', js], capture_output=True, text=True)
    score = json.load(open(js))
    check('7. without --spec the audit is ungraded',
          score['blocking_by'].get('audit') is None
          and any('audit' in str(u) for u in score.get('ungraded', [])), str(score.get('ungraded')))

    print(f"\n{'ALL PASS' if not FAILS else '%d FAIL' % len(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
