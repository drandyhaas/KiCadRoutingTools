#!/usr/bin/env python3
"""#1630: a diff pair in a project's own pair class keeps ONE gap -- the one
KiCad grades it at -- through the floor, the --impedance solve, the build,
the record and the cleanup, and check_impedance discloses the ohms.

The fixture: one straight pair on F.Cu over a GND plane, its class PAIRS at
width 0.15 / gap 0.12 / clearance 0.15, the Default class 0.15 / 0.15 /
0.15, rules.min_clearance 0.09, and (arms A, C) a .kicad_dru rule giving
PAIRS-to-PAIRS 0.11.

  A. --escalation board --no-fix-drc-settings, --impedance 90, the gap
     omitted: the rule is logged at 0.11 (it read 0.15, the Default class
     #1262 floors the clearance BASE at), and the pair is solved and built
     at its class gap 0.12, so the shipped copper reads 90 ohm.
  B. The default policy, no rule, the gap omitted: solved, built and
     recorded at 0.12 (it was solved and recorded at the Default class's
     0.15 and built at 0.12: 86 ohm on the tool's own model).
  C. The rule, the default policy, --diff-pair-gap 0.12: the graze
     micro-shift leaves the pair at 0.12 (it moved N to 0.154).
  D. check_impedance discloses the declared ohms against the shipped copper:
     B within tolerance; a 0.4 mm floor that ships 56 ohm named off target,
     and the exit code unchanged (disclosure only).
  E. fab_tiers.rule_fab_floors: under board with the writeback off, the
     clearance a rule floors at is the board minimum, while fab_floors keeps
     the Default class for the base.

    python3 tests/test_1630_pair_class_one_gap.py
"""
import json
import math
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)
sys.path.insert(0, TESTS_DIR)
for _p in ('py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from run_utils import check as run_check, evidence             # noqa: E402

failures = []
H, T, ER = 0.1, 0.035, 4.3
RULE = ('(version 1)\n(rule "pairs"\n (condition "A.NetClass == \'PAIRS\' && '
        'B.NetClass == \'PAIRS\'")\n (constraint clearance (min 0.11mm)))\n')


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def build(d, rule):
    os.makedirs(d, exist_ok=True)

    def fp(ref, x):
        p = ''.join(f'    (pad "{n}" smd rect (at 0 {yy}) (size 0.5 0.25) (layers "F.Cu")'
                    f' (net {i} "{nm}"))\n'
                    for n, yy, i, nm in (('1', -0.25, 1, '/X_P'), ('2', 0.25, 2, '/X_N')))
        return (f'  (footprint "T:{ref}" (layer "F.Cu") (at {x} 115)\n'
                f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") (hide yes)'
                f' (effects (font (size 1 1) (thickness 0.15))))\n    (attr smd)\n{p}  )\n')
    die = lambda n, ty, th: (f'   (layer "{n}" (type "{ty}") (thickness {th}) (material "FR4")'
                             f' (epsilon_r {ER}) (loss_tangent 0.02))\n')
    cu = lambda n: f'   (layer "{n}" (type "copper") (thickness {T}))\n'
    pts = '(xy 100 100) (xy 150 100) (xy 150 130) (xy 100 130)'
    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal) (2 "B.Cu" signal)'
         ' (25 "Edge.Cuts" user))\n (setup\n  (stackup\n' + cu('F.Cu')
         + die('dielectric 1', 'prepreg', H) + cu('In1.Cu') + die('dielectric 2', 'core', 1.2)
         + cu('In2.Cu') + die('dielectric 3', 'prepreg', H) + cu('B.Cu')
         + '  )\n  (pad_to_mask_clearance 0)\n )\n'
         ' (net 0 "") (net 1 "/X_P") (net 2 "/X_N") (net 3 "GND")\n' + fp('J1', 110) + fp('J2', 140)
         + f'  (zone (net 3) (net_name "GND") (layer "In1.Cu")'
           f' (uuid "11111111-2222-3333-4444-555555555555") (hatch edge 0.5)'
           f' (connect_pads (clearance 0.2)) (min_thickness 0.2) (filled_areas_thickness no)'
           f' (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5)) (polygon (pts {pts}))'
           f' (filled_polygon (layer "In1.Cu") (pts {pts})))\n')
    for x1, y1, x2, y2 in ((100, 100, 150, 100), (150, 100, 150, 130),
                           (150, 130, 100, 130), (100, 130, 100, 100)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts")'
              f' (stroke (width 0.05) (type solid)))\n')
    with open(os.path.join(d, 'x.kicad_pcb'), 'w', encoding='utf-8') as f:
        f.write(b + ')\n')
    cls = lambda n, g: {'name': n, 'clearance': 0.15, 'track_width': 0.15,
                        'via_diameter': 0.45, 'via_drill': 0.2,
                        'diff_pair_width': 0.15, 'diff_pair_gap': g}
    rules = {'min_clearance': 0.09, 'min_track_width': 0.09, 'min_via_diameter': 0.3,
             'min_through_hole_diameter': 0.15, 'min_via_annular_width': 0.05,
             'min_hole_clearance': 0.1, 'min_hole_to_hole': 0.2,
             'min_copper_edge_clearance': 0.2}
    with open(os.path.join(d, 'x.kicad_pro'), 'w', encoding='utf-8') as f:
        json.dump({'meta': {'version': 2},
                   'board': {'design_settings': {'rules': rules}},
                   'net_settings': {'classes': [cls('Default', 0.15), cls('PAIRS', 0.12)],
                                    'netclass_patterns': [{'netclass': 'PAIRS',
                                                           'pattern': '/X_*'}]}}, f)
    if rule:
        with open(os.path.join(d, 'x.kicad_dru'), 'w', encoding='utf-8') as f:
            f.write(RULE)
    return evidence(os.path.join(d, 'x.kicad_pcb'), 'the fixture board')


def route(tmp, tag, rule, *flags):
    board = build(os.path.join(tmp, tag), rule)
    out = os.path.join(tmp, tag, 'o.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'route_diff.py'),
                   board, out, '--nets', '/X_*', '--layers', 'F.Cu', 'B.Cu', *flags],
                  accept=True)
    return evidence(out, f'arm {tag} output'), r.stdout + r.stderr


def shipped(path):
    """(width, P<->N gap) of the longest coupled run (horizontal here)."""
    import contextlib
    import io
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    ids = {n.name: i for i, n in pcb.nets.items()}
    L = lambda nm: max((s for s in pcb.segments if s.net_id == ids[nm]),
                       key=lambda s: math.hypot(s.end_x - s.start_x, s.end_y - s.start_y))
    p, n = L('/X_P'), L('/X_N')
    return p.width, abs(p.start_y - n.start_y) - (p.width + n.width) / 2


def zdiff(w, g):
    from impedance import differential_microstrip_z0
    return differential_microstrip_z0(w, g, H, T, ER)[0]


def main():
    tmp = tempfile.mkdtemp(prefix='t1630_')
    try:
        # A
        out, log = route(tmp, 'a', True, '--impedance', '90', '--no-fix-drc-settings',
                         '--escalation', 'board')
        check('A: the rule is logged at its own 0.11, not the Default class',
              "graded at 0.11mm by the scoped rule 'pairs'" in log,
              (re.search(r'graded at [\d.]+mm[^\n]*', log) or [''])[0])
        w, g = shipped(out)
        check('A: built at the class gap 0.12', abs(g - 0.12) < 0.002, f'gap {g:.4f}')
        check('A: solved there, so the copper reads 90 ohm',
              abs(zdiff(w, g) - 90) < 1.0, f'width {w:.4f} -> {zdiff(w, g):.1f} ohm')

        # B
        out_b, log = route(tmp, 'b', False, '--impedance', '90')
        w, g = shipped(out_b)
        check('B: built at the class gap 0.12', abs(g - 0.12) < 0.002, f'gap {g:.4f}')
        check('B: solved at the gap it is built at', abs(zdiff(w, g) - 90) < 1.0,
              f'width {w:.4f} -> {zdiff(w, g):.1f} ohm')
        with open(os.path.join(tmp, 'b', 'o.kicad_pro'), encoding='utf-8') as f:
            rec = json.load(f)['kicad_routing_tools']['net_impedance']['/X_P']
        check('B: and recorded at it', abs(rec.get('pair_gap', 0) - 0.12) < 1e-6, str(rec))

        # C
        out, log = route(tmp, 'c', True, '--impedance', '90', '--diff-pair-gap', '0.12')
        w, g = shipped(out)
        check('C: the cleanup leaves a rule-graded pair at its gap',
              abs(g - 0.12) < 0.002, f'gap {g:.4f}')
        check('C: no micro-shift moved it', 'Graze micro-shift' not in log)

        # D
        tool = os.path.join(ROOT, 'py_tools', 'check_impedance.py')
        jb = os.path.join(tmp, 'b', 'imp.json')
        run_check([sys.executable, '-X', 'utf8', tool, out_b, '--json', jb], accept=True)
        with open(jb, encoding='utf-8') as f:
            d = json.load(f)
        rows = {r['net_name']: r for r in d['nets']}
        check('D: the declared ohms are read on the shipped copper',
              abs((rows['/X_P'].get('shipped_ohms') or 0) - 90) < 2.0,
              str({k: rows['/X_P'].get(k) for k in ('declared_ohms', 'shipped_ohms',
                                                    'ohms_off_pct')}))
        check('D: B is not off target', not d['totals']['nets_ohms_off_target'],
              str(d['totals']['nets_ohms_off_target']))
        out_d, _ = route(tmp, 'd', False, '--impedance', '90', '--track-width', '0.4',
                         '--diff-pair-gap', '0.2')
        jd = os.path.join(tmp, 'd', 'imp.json')
        r = run_check([sys.executable, '-X', 'utf8', tool, out_d, '--json', jd], accept=True)
        with open(jd, encoding='utf-8') as f:
            d = json.load(f)
        check('D: a 0.4 mm floor far off 90 ohm is named off target',
              d['totals']['nets_ohms_off_target'] == ['/X_N', '/X_P'],
              str(d['totals']['nets_ohms_off_target']))
        check('D: and printed as such', 'OFF TARGET' in r.stdout)

        # E
        import fab_tiers
        from types import SimpleNamespace as NS
        prev = fab_tiers.get_escalation_policy()
        try:
            board = os.path.join(tmp, 'a', 'x.kicad_pcb')
            import contextlib
            import io
            with contextlib.redirect_stdout(io.StringIO()):
                fab_tiers.set_policy_from_args(
                    NS(escalation='board', no_fix_drc_settings=True), board)
            base = fab_tiers.fab_floors(4)['clearance']
            rule = fab_tiers.rule_fab_floors(4)['clearance']
            check('E: the base floor is the Default class', abs(base - 0.15) < 1e-9, f'{base}')
            check('E: a rule floors at the board minimum, not the class',
                  rule < 0.11 - 1e-9, f'{rule}')
            with contextlib.redirect_stdout(io.StringIO()):
                fab_tiers.set_policy_from_args(
                    NS(escalation='board', no_fix_drc_settings=False), board)
            check('E: with the writeback, both read the same',
                  fab_tiers.rule_fab_floors(4) == fab_tiers.fab_floors(4))
        finally:
            fab_tiers.set_escalation_policy(*prev)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f"FAIL: {len(failures)}: {failures}")
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
