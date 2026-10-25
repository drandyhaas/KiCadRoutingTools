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
   and max_vias; a pin that does not exist fails (a finding about the spec).
5. pairs on a stackup board: each coupled stretch's impedance equals
   impedance.calculate_impedance_for_layer at its width and gap; the window
   from gap_max_mm leaves a wider stretch out; skew and uncoupled length;
   no stackup is not_judged. pair.reference: the pair over a GND pour passes,
   and with reference_nets naming another net the same pour does not count.
6. length.match and net.min_width; the board's own min_track_width wins when
   it is stricter than the spec.
7. board_score --spec counts the failed SPEC checks in blocking, never the
   spec-free ones; without --spec the audit component is ungraded.
8. The review of #1237, one case per finding:
   (1) a via on a board with no stackup is rated, its barrel the board
       thickness; (3) an unrouted pair's uncoupled length is not_judged;
   (4) a file that is not a board is exit 3, as a baseline exit 2;
   (5) a value of the wrong type or range is exit 2, never a crash or a
       character-by-character read; (6) a reference net not on the board is
       a failed finding about the spec (since section 11); (7) one through via on four layers is one via;
   (8) a terminal's pads that share a number conduct together; (9) a pad
       with no number is no default load; (10) a width glob matching nothing
       is reported, a length group's missing net fails, tolerance_mm is
       optional; (11) one finding per segment and per via; (12) a via step's
       barrel is the stackup's length between its two layers, less half of
       the copper of an end layer that is not one of the via's own ends, so
       a via's steps add up to its barrel.
9. The review of the fixes: an empty reference_nets and an integral float
   max_vias are valid; a glob shadowed by an earlier one gets no "matches
   nothing" record; a binary file and a board cut short are exit 3 (exit 2
   as a baseline).
10. The review of those: a teardrop child without its "(" (#1149) and a
   leading byte-order mark still audit; a blind or buried via's steps add
   up to its barrel; an integer too large for a float is exit 2.
11. The whole-PR review: a name the spec gives that the board lacks (pair
   net, current net/source/load, decoupling cap) fails; a routed pair
   entirely outside its gap window fails pair.impedance at its closest
   stretch, an unrouted one is not judged; board_score grades no audit when
   no spec check was judged (an empty spec); "_" keys in net_min_widths are
   comments; a one-net length group fails; converge compares a score from
   before the audit component with one after.
12. Pours (plane_fill_model's fill as a resistor grid): a strip pour fed by
   full-width pads drops rho L / (W t); two overlapping pours of the net
   on one layer are one strip; two pours that do not touch leave the load
   unjoined; a via joins pours on two layers, above the plain strip and
   barrel sum (spreading at the via), and without it the load is unjoined;
   a pour is not rated, so current that runs only through one leaves
   capacity not judged; decoupling walks a pour, straight and diagonally
   (octile: never short, at most 9 % long).

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

STACKUP4 = ('(stackup (layer "F.Cu" (type "copper") (thickness 0.035))'
            ' (layer "dielectric 1" (type "prepreg") (thickness 0.2) (epsilon_r 4.5))'
            ' (layer "In1.Cu" (type "copper") (thickness 0.035))'
            ' (layer "dielectric 2" (type "core") (thickness 1.0) (epsilon_r 4.5))'
            ' (layer "In2.Cu" (type "copper") (thickness 0.035))'
            ' (layer "dielectric 3" (type "prepreg") (thickness 0.2) (epsilon_r 4.5))'
            ' (layer "B.Cu" (type "copper") (thickness 0.035)))')


def board4(body, name):
    text = ('(kicad_pcb (version 20240202) (generator "t")\n'
            ' (general (thickness 1.54))\n'
            ' (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal)'
            ' (31 "B.Cu" signal) (39 "F.Mask" user) (38 "B.Mask" user) (44 "Edge.Cuts" user))\n'
            ' (setup %s)\n'
            ' (net 0 "") (net 3 "/A")\n'
            ' (gr_rect (start 80 80) (end 140 120) (layer "Edge.Cuts") (width 0.1))\n'
            '%s\n)\n') % (STACKUP4, body)
    path = os.path.join(WORK, name + '.kicad_pcb')
    with open(path, 'w') as fh:
        fh.write(text)
    return path


def review_cases():
    # (1) no stackup: the via is still rated, at the board thickness
    nostack_via = board(pad('J1', 100, 100, 3, '/A') + pad('U1', 110, 100, 3, '/A', layers='"B.Cu" "B.Mask"')
                        + seg(100, 100, 105, 100, 3) + via(105, 100, 3, drill=0.3)
                        + seg(105, 100, 110, 100, 3, layer='B.Cu'), stackup=False, name='nsv')
    rc, doc, _o = run(nostack_via, {'currents': {'/A': {'amps': 50, 'source': 'J1.1'}}})
    c = get(doc, 'current.via', '/A')
    check('8(1). no stackup: a via over its rating fails', c['status'] == 'failed', str(c))
    check('8(1). ...and the barrel assumption is stated',
          any('board thickness' in a for a in c.get('assumptions', [])), str(c.get('assumptions')))
    # (3) a pair with pads only
    pads_only = board(pad('J1', 100, 100, 1, '/P') + pad('U1', 110, 100, 1, '/P')
                      + pad('J2', 100, 102, 5, '/N') + pad('U2', 110, 102, 5, '/N'), name='po')
    rc, doc, _o = run(pads_only, {'pairs': {'D': {'nets': ['/P', '/N'], 'max_uncoupled_mm': 1}}})
    check('8(3). an unrouted pair: uncoupled is not_judged',
          get(doc, 'pair.uncoupled', 'D')['status'] == 'not_judged')
    # (4) not a board
    junk = os.path.join(WORK, 'junk.kicad_pcb')
    with open(junk, 'w') as fh:
        fh.write('not a board')
    check('8(4). a file that is not a board is exit 3', run(junk)[0] == 3)
    check('8(4). ...and as a baseline exit 2', run(plain_board(), extra=['--baseline', junk])[0] == 2)
    # (5) values
    cur = {'amps': 1, 'source': 'J1.1'}
    bad_values = [
        ('amps "abc"', {'currents': {'/A': dict(cur, amps='abc')}}),
        ('amps -1', {'currents': {'/A': dict(cur, amps=-1)}}),
        ('max_rise_c null', {'currents': {'/A': dict(cur, max_rise_c=None)}}),
        ('loads {U1.1: "x"}', {'currents': {'/A': dict(cur, loads={'U1.1': 'x'})}}),
        ('loads "U1.1"', {'currents': {'/A': dict(cur, loads='U1.1')}}),
        ('source 5', {'currents': {'/A': dict(cur, source=5)}}),
        ('tolerance_mm "x"', {'length_groups': {'G': {'nets': ['/A'], 'tolerance_mm': 'x'}}}),
        ('mode "bogus"', {'length_groups': {'G': {'nets': ['/A'], 'mode': 'bogus'}}}),
        ('nets "/A"', {'length_groups': {'G': {'nets': '/A'}}}),
        ('max_vias "x"', {'decoupling': {'C1': {'ic': 'U1', 'pins': ['1'],
                                                'max_distance_mm': 1, 'max_vias': 'x'}}}),
        ('pins "12"', {'decoupling': {'C1': {'ic': 'U1', 'pins': '12', 'max_distance_mm': 1}}}),
        ('max_skew_mm "x"', {'pairs': {'D': {'nets': ['/A', '/B'], 'max_skew_mm': 'x'}}}),
        ('net_min_widths -1', {'net_min_widths': {'/A': -1}}),
        ('net_min_widths true', {'net_min_widths': {'/A': True}}),
    ]
    for what, spec in bad_values:
        rc, _d, out = run(plain_board(), spec)
        check(f'8(5). {what} is exit 2', rc == 2, out[-200:])
    # (6) a reference net not on the board
    rc, doc, _o = run(os.path.join(WORK, 'pair.kicad_pcb'),
                      {'pairs': {'D': {'nets': ['/P', '/N']}}, 'reference_nets': ['GNDX']})
    c = get(doc, 'pair.reference', 'D')
    check('8(6). reference_nets naming no net fails, a finding about the spec',
          c['status'] == 'failed' and 'GNDX' in c['findings'][0]['reason'], str(c))
    # (7) + (12) one through via on four layers
    four = board4(pad('C1', 100, 100, 3, '/A') + pad('U1', 104, 100, 3, '/A', layers='"B.Cu" "B.Mask"')
                  + seg(100, 100, 102, 100, 3) + via(102, 100, 3)
                  + seg(102, 100, 104, 100, 3, layer='B.Cu'), 'four')
    rc, doc, _o = run(four, {'decoupling': {'C1': {'ic': 'U1', 'pins': ['1'],
                                                   'max_distance_mm': 10, 'max_vias': 1}}})
    c = get(doc, 'decoupling.path', 'C1')
    check('8(7). one through via on four layers is one via',
          c['status'] == 'passed' and c['measured']['U1.1']['vias'] == 1, str(c))
    rc, doc, _o = run(four, {'currents': {'/A': {'amps': 50, 'source': 'C1.1'}}})
    c = get(doc, 'current.via', '/A')
    check('8(11). a through via over its rating is one finding',
          c['status'] == 'failed' and len(c['findings']) == 1, str(c))
    part = board4(pad('C1', 100, 100, 3, '/A') + pad('U1', 104, 100, 3, '/A', layers='"F.Cu" "F.Mask"')
                  + seg(100, 100, 102, 100, 3) + via(102, 100, 3)
                  + seg(102, 100, 103, 100, 3, layer='In1.Cu') + via(103, 100, 3)
                  + seg(103, 100, 104, 100, 3), 'part')
    pcb = parse_kicad_pcb(part)
    step = pcb.get_via_barrel_length('F.Cu', 'In1.Cu') - 0.035 / 2   # In1.Cu is inner
    v_ohm = RHO_OHM_MM * step / (math.pi * 0.3 * dc_network.VIA_PLATING_MM)
    want = (RHO_OHM_MM * 4 / (0.5 * T_CU) + 2 * v_ohm) * 1e3   # 2 + 1 + 1 mm of track
    rc, doc, _o = run(part, {'currents': {'/A': {'amps': 1, 'source': 'C1.1', 'max_drop_mv': 1000}}})
    d = get(doc, 'current.drop', '/A')['measured']['U1.1']
    check(f'8(12). an F.Cu-In1.Cu step is charged its {step:.4f} mm barrel',
          abs(d - want) < 0.005, f'{d} vs {want:.4f}')
    full = pcb.get_via_barrel_length('F.Cu', 'B.Cu')
    v_full = RHO_OHM_MM * full / (math.pi * 0.3 * dc_network.VIA_PLATING_MM)
    want = (RHO_OHM_MM * 4 / (0.5 * T_CU) + v_full) * 1e3
    rc, doc, _o = run(four, {'currents': {'/A': {'amps': 1, 'source': 'C1.1', 'max_drop_mv': 1000}}})
    d = get(doc, 'current.drop', '/A')['measured']['U1.1']
    check(f"8(12). a through via's three steps add up to its {full:.3f} mm barrel",
          abs(d - want) < 0.005, f'{d} vs {want:.4f}')
    # (8) repeated pad numbers
    two_ones = ('(footprint "L:P" (layer "F.Cu") (at 100 100) (property "Reference" "J1")'
                ' (pad "1" smd rect (at 0 -1) (size 1 1) (layers "F.Cu" "F.Mask") (net 3 "/A"))'
                ' (pad "1" smd rect (at 0 1) (size 1 1) (layers "F.Cu" "F.Mask") (net 3 "/A")))\n')
    second = board(two_ones + pad('U1', 110, 101, 3, '/A') + seg(100, 101, 110, 101, 3), name='second')
    rc, doc, _o = run(second, {'currents': {'/A': {'amps': 1, 'source': 'J1.1', 'max_drop_mv': 1000}}})
    d = get(doc, 'current.drop', '/A')
    one = RHO_OHM_MM * 10 / (0.5 * T_CU)
    check("8(8). a track to the source's second '1' pad is judged",
          d['status'] != 'not_judged' and abs(d['measured']['U1.1'] - one * 1e3) < 0.01, str(d))
    both = board(two_ones + pad('U1', 110, 100, 3, '/A', size='1 3')
                 + seg(100, 99, 110, 99, 3) + seg(100, 101, 110, 101, 3), name='both')
    rc, doc, _o = run(both, {'currents': {'/A': {'amps': 1, 'source': 'J1.1', 'max_drop_mv': 1000}}})
    d = get(doc, 'current.drop', '/A')['measured']['U1.1']
    check("8(8). both '1' pads tracked in parallel halve the drop",
          abs(d - one * 1e3 / 2) < 0.01, f'{d} vs {one * 1e3 / 2:.3f}')
    # (9) a numberless pad
    numberless = board(pad('J1', 100, 100, 3, '/A') + pad('U1', 110, 100, 3, '/A')
                       + pad('H1', 120, 100, 3, '/A', num='') + seg(100, 100, 110, 100, 3),
                       name='numberless')
    rc, doc, _o = run(numberless, {'currents': {'/A': {'amps': 0.1, 'source': 'J1.1'}}})
    check('8(9). a pad with no number is not a default load',
          get(doc, 'current.capacity', '/A')['status'] == 'passed',
          str(get(doc, 'current.capacity', '/A')))
    # (10) records for everything the spec names
    rc, doc, _o = run(plain_board(), {'net_min_widths': {'/NOPE*': 0.2}})
    c = get(doc, 'net.min_width', '/NOPE*')
    check('8(10). a width glob that matches nothing is not_judged, not absent',
          c is not None and c['status'] == 'not_judged', str(doc['checks'][-1]))
    rc, doc, _o = run(plain_board(), {'length_groups': {'G': {'nets': ['/A', '/MISSING']}}})
    check("8(10). a length group's missing net fails, as board_score counts it",
          get(doc, 'length.match', 'G')['status'] == 'failed')
    rc, doc, _o = run(os.path.join(WORK, 'pair.kicad_pcb'),
                      {'length_groups': {'G': {'nets': ['/P', '/N']}}})
    check('8(10). tolerance_mm is optional (0.1 mm, as board_score)',
          get(doc, 'length.match', 'G')['status'] == 'passed')
    # (11) one finding per segment, however the graph cuts it
    tee = board(pad('J1', 100, 100, 3, '/A') + pad('U1', 110, 100, 3, '/A') + pad('U2', 105, 105, 3, '/A')
                + seg(100, 100, 110, 100, 3, w=0.1) + seg(105, 100, 105, 105, 3, w=0.1), name='tee')
    rc, doc, _o = run(tee, {'currents': {'/A': {'amps': 5, 'source': 'J1.1', 'loads': ['U1.1']}}})
    c = get(doc, 'current.capacity', '/A')
    check('8(11). a segment cut by a T-junction is one capacity finding',
          c['status'] == 'failed' and len(c['findings']) == 1, str(c))


def fix_review_cases():
    rc, _d, out = run(os.path.join(WORK, 'pair.kicad_pcb'),
                      {'pairs': {'D': {'nets': ['/P', '/N']}}, 'reference_nets': []})
    check('9. an empty reference_nets is valid (any pour counts)', rc in (0, 4), out[-200:])
    rc, _d, out = run(os.path.join(WORK, 'dec.kicad_pcb'),
                      {'decoupling': {'C1': {'ic': 'U1', 'pins': ['1'], 'max_distance_mm': 9,
                                             'max_vias': 2.0}}})
    check('9. max_vias 2.0 is valid', rc in (0, 4), out[-200:])
    rc, _d, out = run(os.path.join(WORK, 'dec.kicad_pcb'),
                      {'decoupling': {'C1': {'ic': 'U1', 'pins': ['1'], 'max_distance_mm': 9,
                                             'max_vias': 1.5}}})
    check('9. max_vias 1.5 is not', rc == 2, out[-200:])
    rc, doc, _o = run(plain_board(), {'net_min_widths': {'/A': 0.1, '/*': 0.3}})
    check('9. a glob shadowed by an earlier one gets no "matches nothing" record',
          get(doc, 'net.min_width', '/*') is None, str(doc['checks'][-2:]))
    binfile = os.path.join(WORK, 'bin.kicad_pcb')
    with open(binfile, 'wb') as fh:
        fh.write(bytes(range(256)) * 12)
    check('9. a binary file is exit 3', run(binfile)[0] == 3)
    check('9. ...and as a baseline exit 2', run(plain_board(), extra=['--baseline', binfile])[0] == 2)
    cut = os.path.join(WORK, 'cut.kicad_pcb')
    with open(cut, 'w') as fh:
        fh.write(open(os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')).read()[:5000])
    check('9. a board cut short is exit 3', run(cut)[0] == 3)


TD_BAD = ('(teardrops (best_length_ratio 0.5) (max_length 1) (best_width_ratio 1) '
          '(max_width 2) (curved_edges no)filter_ratio 0.9) (enabled yes))')


def second_fix_review_cases():
    plain = open(plain_board()).read()
    td = os.path.join(WORK, 'teardrop.kicad_pcb')
    with open(td, 'w') as fh:
        fh.write(plain.replace('(net 3 "/A")))', '(net 3 "/A") %s))' % TD_BAD))
    rc, _d, out = run(td)
    check('10. a teardrop child without its "(" still audits', rc == 0, out[-300:])
    bom = os.path.join(WORK, 'bom.kicad_pcb')
    with open(bom, 'w', encoding='utf-8') as fh:
        fh.write('\ufeff' + plain)
    rc, _d, out = run(bom)
    check('10. a leading byte-order mark still audits', rc == 0, out[-300:])
    huge = 10 ** 400
    for spec in ({'decoupling': {'C1': {'ic': 'U1', 'pins': ['1'], 'max_distance_mm': 9,
                                        'max_vias': huge}}},
                 {'decoupling': {'C1': {'ic': 'U1', 'pins': ['1'], 'max_distance_mm': huge}}}):
        rc, _d, out = run(os.path.join(WORK, 'dec.kicad_pcb'), spec)
        check('10. an integer too large for a float is exit 2', rc == 2, out[-200:])
    for name, (l1, l2) in (('blind', ('F.Cu', 'In2.Cu')), ('buried', ('In1.Cu', 'In2.Cu'))):
        v = ('(via %s (at 102 100) (size 0.6) (drill 0.3) (layers "%s" "%s") (net 3))\n'
             % ('blind' if name == 'blind' else 'buried', l1, l2))
        body = (pad('C1', 100, 100, 3, '/A', layers='"%s"' % l1)
                + pad('U1', 104, 100, 3, '/A', layers='"%s"' % l2)
                + seg(100, 100, 102, 100, 3, layer=l1) + v + seg(102, 100, 104, 100, 3, layer=l2))
        pcb = parse_kicad_pcb(board4(body, name))
        net = dc_network.build(pcb, 3, {'C1.1': [pcb.footprints['C1'].pads[0]],
                                        'U1.1': [pcb.footprints['U1'].pads[0]]})
        steps = sum(e.length_mm for e in net.edges if e.kind == 'via')
        full = pcb.get_via_barrel_length(l1, l2)
        check(f"10. a {name} via's steps add up to its {full:.4f} mm barrel",
              abs(steps - full) < 1e-9, f'{steps:.4f}')


def full_review_cases():
    from impedance import calculate_impedance_for_layer
    pair_b, dec_b = os.path.join(WORK, 'pair.kicad_pcb'), os.path.join(WORK, 'dec.kicad_pcb')
    rc, doc, _o = run(pair_b, {'pairs': {'D': {'nets': ['/P', '/USB_DM'], 'max_skew_mm': 1}}})
    c = get(doc, 'pair.skew', 'D')
    check('11. a pair net not on the board fails', rc == 4 and c['status'] == 'failed', str(c))
    for what, entry in (('a current net', ('/NOPE', {'amps': 1, 'source': 'J1.1'})),
                        ('a current source', ('/A', {'amps': 1, 'source': 'J9.1'})),
                        ('a current load', ('/A', {'amps': 1, 'source': 'J1.1', 'loads': ['U9.1']}))):
        rc, doc, _o = run(plain_board(), {'currents': {entry[0]: entry[1]}})
        c = get(doc, 'current.capacity', entry[0])
        check(f'11. {what} not on the board fails', c['status'] == 'failed', str(c))
    rc, doc, _o = run(dec_b, {'decoupling': {'C9': {'ic': 'U1', 'pins': ['1'], 'max_distance_mm': 5}}})
    check('11. a decoupling cap not on the board fails',
          get(doc, 'decoupling.path', 'C9')['status'] == 'failed')
    # a routed pair whose only stretch is outside its window
    wide = board(pad('J1', 100, 100, 1, '/P', size='0.2 0.2') + pad('J2', 100, 100.8, 5, '/N', size='0.2 0.2')
                 + pad('U1', 120, 100, 1, '/P', size='0.2 0.2') + pad('U2', 120, 100.8, 5, '/N', size='0.2 0.2')
                 + seg(100, 100, 120, 100, 1, w=0.2) + seg(100, 100.8, 120, 100.8, 5, w=0.2), name='wide')
    z = calculate_impedance_for_layer(parse_kicad_pcb(wide), 'F.Cu', 0.2, spacing=0.6)['zdiff']
    for gap in ({'gap_max_mm': 0.2}, {}):
        rc, doc, _o = run(wide, {'pairs': {'D': dict({'nets': ['/P', '/N'], 'impedance_ohm': 90,
                                                      'tolerance_pct': 10}, **gap)}})
        c = get(doc, 'pair.impedance', 'D')
        check(f'11. a pair routed outside its window {gap or "(dielectric height)"} '
              f'fails at its closest stretch, {z:.1f} ohm',
              c['status'] == 'failed' and c['findings'][0]['measured'] == round(z, 2), str(c))
    bare = board(pad('J1', 100, 100, 1, '/P') + pad('J2', 100, 101, 5, '/N')
                 + pad('U1', 120, 100, 1, '/P') + pad('U2', 120, 101, 5, '/N'), name='bare')
    rc, doc, _o = run(bare, {'pairs': {'D': {'nets': ['/P', '/N'], 'impedance_ohm': 90,
                                             'tolerance_pct': 10}}})
    check('11. an unrouted pair is not judged for impedance',
          get(doc, 'pair.impedance', 'D')['status'] == 'not_judged')
    rc, doc, out = run(plain_board(), {'net_min_widths': {'_comment': 'rails', '/A': 0.4}})
    check('11. a "_" key in net_min_widths is a comment', rc in (0, 4)
          and get(doc, 'net.min_width', '_comment') is None, out[-200:])
    rc, doc, _o = run(plain_board(), {'length_groups': {'G': {'nets': ['/A']}}})
    check('11. a one-net length group fails, as board_score counts it',
          get(doc, 'length.match', 'G')['status'] == 'failed')
    bs = os.path.join(ROOT, 'py_tools', 'board_score.py')
    js = os.path.join(WORK, 'score11.json')
    unrouted_pair = {'pairs': {'D': {'nets': ['/P', '/N'], 'impedance_ohm': 90, 'tolerance_pct': 10}}}
    for what, target, spec in (('an empty spec', plain_board(), {}),
                               ('a spec judging nothing (an unrouted pair)', bare, unrouted_pair)):
        subprocess.run([sys.executable, bs, target, '--spec', spec_file(spec, 'e11'), '--json', js],
                       capture_output=True, text=True)
        score = json.load(open(js))
        comp = score['components']['audit']
        check(f'11. board_score grades no audit for {what}',
              comp['ran'] is False and comp['count'] is None and 'audit' in score['ungraded'],
              str(comp))
    sp = spec_file({'pairs': {'U': {'nets': ['/USB_DP', '/USB_DM'], 'max_skew_mm': 0.1}}}, 'typo')
    subprocess.run([sys.executable, bs, plain_board(), '--spec', sp, '--json', js],
                   capture_output=True, text=True)
    check('11. a spec whose names are not on the board blocks (skew and reference)',
          json.load(open(js))['blocking_by'].get('audit') == 2)
    sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
    from converge import commensurability
    before = {'blocking': 5, 'blocking_by': {'unrouted': 5}, 'ungraded': [], 'components': {}}
    after = {'blocking': 4, 'blocking_by': {'unrouted': 4, 'audit': None}, 'ungraded': ['audit'],
             'components': {'audit': {'ran': False, 'reason': 'no --spec given'}}}
    check('11. converge: a score from before the audit component compares with one after',
          commensurability(before, after) is None, str(commensurability(before, after)))
    graded = {'blocking': 5, 'blocking_by': {'unrouted': 4, 'audit': 1}, 'ungraded': [],
              'components': {}}
    lost = {'blocking': 4, 'blocking_by': {'unrouted': 4}, 'ungraded': [], 'components': {}}
    got = commensurability(graded, lost)
    check('11. converge: a score that drops a graded audit is still a false improvement',
          got is not None and got[0] == ['audit'] and got[2] is True, str(got))


def zone(net, name, x0, y0, x1, y1, layer='F.Cu'):
    return ('(zone (net %d) (net_name "%s") (layer "%s") (hatch edge 0.5)'
            ' (connect_pads (clearance 0.2)) (min_thickness 0.1) (fill yes)'
            ' (polygon (pts (xy %g %g) (xy %g %g) (xy %g %g) (xy %g %g))))\n'
            % (net, name, layer, x0, y0, x1, y0, x1, y1, x0, y1))


def pour_cases():
    ends = pad('J1', 100.5, 100, 3, '/A', size='1 4')
    far = pad('U1', 119.5, 100, 3, '/A', size='1 4')
    spec = {'currents': {'/A': {'amps': 1, 'source': 'J1.1', 'loads': ['U1.1'],
                                'max_drop_mv': 1000}}}
    strip = RHO_OHM_MM * 18 / (4 * T_CU) * 1e3          # mV between the pads
    for what, body in (('one strip pour', zone(3, '/A', 100, 98, 120, 102)),
                       ('two overlapping pours', zone(3, '/A', 100, 98, 110.5, 102)
                        + zone(3, '/A', 109.5, 98, 120, 102))):
        rc, doc, _o = run(board(ends + far + body, name='strip'), spec)
        d = get(doc, 'current.drop', '/A')
        check(f'12. {what} 4 x 18 mm drops rho L / (W t) = {strip:.3f} mV',
              d['status'] == 'passed' and abs(d['measured']['U1.1'] - strip) < 0.01 * strip, str(d))
    c = get(doc, 'current.capacity', '/A')
    check('12. ...a pour is not rated: capacity is not judged',
          c['status'] == 'not_judged' and 'pour' in c['reason'], str(c))
    check('12. ...and the pour model is disclosed',
          any('plane_fill_model' in a for a in c['assumptions']), str(c['assumptions']))
    rc, doc, _o = run(board(ends + far + zone(3, '/A', 100, 98, 109.5, 102)
                            + zone(3, '/A', 110.5, 98, 120, 102), name='split'), spec)
    check('12. two pours that do not touch leave the load unjoined',
          get(doc, 'current.drop', '/A')['status'] == 'not_judged')
    back = pad('U1', 119.5, 100, 3, '/A', size='1 4', layers='"B.Cu" "B.Mask"')
    two = ends + back + zone(3, '/A', 100, 98, 110.5, 102) + zone(3, '/A', 109.5, 98, 120, 102,
                                                                 layer='B.Cu')
    floor = strip + RHO_OHM_MM * parse_kicad_pcb(board(two, name='nv')).get_via_barrel_length(
        'F.Cu', 'B.Cu') / (math.pi * 0.3 * dc_network.VIA_PLATING_MM) * 1e3
    rc, doc, _o = run(board(two + via(110, 100, 3), name='twolayer'), spec)
    d = get(doc, 'current.drop', '/A')
    check(f'12. a via joins two layers\' pours, above the {floor:.3f} mV strip + barrel',
          d['status'] == 'passed' and floor < d['measured']['U1.1'] < 1.5 * floor, str(d))
    rc, doc, _o = run(board(two, name='noview'), spec)
    check('12. ...and without the via the load is unjoined',
          get(doc, 'current.drop', '/A')['status'] == 'not_judged')
    for what, (x, y), straight in (('straight', (110, 100), 9.0),
                                   ('diagonal', (108, 106), math.hypot(7, 5))):
        b = board(pad('C1', 100, 100, 3, '/A') + pad('U1', x, y, 3, '/A')
                  + zone(3, '/A', 98, 98, 112, 108), name='dec_' + what)
        rc, doc, _o = run(b, {'decoupling': {'C1': {'ic': 'U1', 'pins': ['1'],
                                                    'max_distance_mm': 20}}})
        c = get(doc, 'decoupling.path', 'C1')
        got = c.get('measured', {}).get('U1.1', {}).get('path_mm')
        check(f'12. decoupling walks a pour {what}: {straight:.2f} mm edge to edge',
              got is not None and straight - 0.21 <= got <= straight * 1.09, str(c))


def plain_board():
    return os.path.join(WORK, 'plain.kicad_pcb')
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
    check('4. a pin that does not exist fails',
          get(doc, 'decoupling.path', 'C1')['status'] == 'failed')

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
    c = get(doc, 'pair.impedance', 'D')
    check('5. a gap window below the 0.15 mm gap: the routed pair is never coupled, so fails',
          c['status'] == 'failed' and c['findings'][0]['measured'] == round(z, 2), str(c))
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
    jog = board(pad('J1', 100, 100, 3, '/A') + pad('U1', 108, 101.5, 3, '/A')
                + seg(100, 100, 104, 100, 3) + seg(104, 100, 106.598, 101.5, 3)
                + seg(106.598, 101.5, 108, 101.5, 3), name='jog')
    sp = spec_file({'net_min_widths': {'/A': 0.1}}, 'bs2')
    subprocess.run([sys.executable, bs, jog, '--spec', sp, '--json', js],
                   capture_output=True, text=True)
    comp = json.load(open(js))['components']['audit']
    check('7. (review 2) a spec-free failure is listed, never counted',
          comp['count'] == 0 and 'weird.off_angle_corner' in comp['spec_free_failed'], str(comp))
    subprocess.run([sys.executable, bs, plain, '--json', js], capture_output=True, text=True)
    score = json.load(open(js))
    check('7. without --spec the audit is ungraded',
          score['blocking_by'].get('audit') is None
          and any('audit' in str(u) for u in score.get('ungraded', [])), str(score.get('ungraded')))

    review_cases()
    fix_review_cases()
    second_fix_review_cases()
    full_review_cases()
    pour_cases()

    print(f"\n{'ALL PASS' if not FAILS else '%d FAIL' % len(FAILS)}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
