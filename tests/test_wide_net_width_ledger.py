"""Impedance- and class-width necks reach the design_rules ledger, per shipped
copper (found reviewing #1282).

A net whose own width is wider than the run's track width -- an impedance
width (--impedance) or a wider net class -- necks through the same ladder as
a power net, and the ledger never said so: the ladder records nothing, and
#1033's shipped-copper rows cover --power-nets only. A run that shipped 8 mm
of a 0.27 mm (50 ohm) net at 0.15 mm exited 0 under --strict-sizes. And the
rows other sites DO record for such a net (terminal neck, net rescue, plane
tap) are per ATTEMPT, so a tap that never shipped could fail --strict-sizes.

route.py now measures, on the board it ships, the copper THIS run laid for
every such net, per layer, and replaces the net's track_width rows with one
`impedance copper shipped under width` / `class-width copper shipped under
width` row. repair_planes.py does the same for its --power-nets taps.

Board: two SIG pads 14 mm apart on F.Cu (routed on F.Cu only) and a foreign
wall across the board with a 0.5 mm gap: a 0.15 mm track threads it at 0.15
clearance, a 0.27 mm one does not, so the route necks to 0.15.

  * --impedance 50: exit 3 under --strict-sizes, the row named (FAILS on
    bb5943a0: exit 0);
  * a 0.4 mm net class on SIG, no --track-width: likewise;
  * controls -- a 3 mm gap (no neck): exit 0, no line; the same with a
    0.1 mm SIG stub already on the INPUT board: still exit 0 -- copper an
    earlier step laid is not this run's neck;
  * repair_planes --power-nets GND 1.0 on a poured board: exit 3, ONE
    `power copper shipped under width` row and no per-attempt `plane tap`
    rows (bb5943a0: exit 0, five `plane tap` rows in its ledger);
  * the ledger rule itself: per-attempt rows of a net that ships nothing
    narrow are dropped, not kept.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check, evidence, tool  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
LINE = 'Design rules ['


def _pad(ref, x, y, net):
    return f'''\t(footprint "T:Pad"
\t\t(layer "F.Cu")
\t\t(uuid "{uuid.uuid4()}")
\t\t(at {x} {y})
\t\t(property "Reference" "{ref}" (at 0 -1.5 0) (layer "F.SilkS") (uuid "{uuid.uuid4()}")
\t\t\t(effects (font (size 1 1) (thickness 0.15))))
\t\t(attr smd)
\t\t(pad "1" smd circle (at 0 0) (size 0.8 0.8) (layers "F.Cu" "F.Mask")
\t\t\t(net "{net}") (uuid "{uuid.uuid4()}"))
\t)'''


def _seg(x1, y1, x2, y2, w, net):
    return (f'\t(segment (start {x1} {y1}) (end {x2} {y2}) (width {w}) '
            f'(layer "F.Cu") (net "{net}") (uuid "{uuid.uuid4()}"))')


def _board(path, wall_gap=0.5, stub=False, netclass=False):
    half = wall_gap / 2 + 0.1        # wall centreline; its width is 0.2
    parts = ['''(kicad_pcb
\t(version 20260206)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general (thickness 1.6) (legacy_teardrops no))
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(1 "F.Mask" user)
\t\t(3 "B.Mask" user)
\t\t(5 "F.SilkS" user "F.Silkscreen")
\t\t(7 "B.SilkS" user "B.Silkscreen")
\t\t(25 "Edge.Cuts" user)
\t\t(35 "F.Fab" user)
\t\t(33 "B.Fab" user)
\t)
\t(setup
\t\t(stackup
\t\t\t(layer "F.Cu" (type "copper") (thickness 0.035))
\t\t\t(layer "dielectric 1" (type "core") (thickness 0.15) (material "FR4") (epsilon_r 4.5) (loss_tangent 0.02))
\t\t\t(layer "B.Cu" (type "copper") (thickness 0.035))
\t\t)
\t\t(pad_to_mask_clearance 0)
\t)''', _pad('U1', 3, 0, 'SIG'), _pad('U2', 17, 0, 'SIG'),
             _pad('U3', 10, -9, 'WALL'),
             _seg(10, -10, 10, -half, 0.2, 'WALL'),
             _seg(10, half, 10, 10, 0.2, 'WALL')]
    if stub:
        parts.append(_seg(3, 0, 4, 0, 0.1, 'SIG'))
    for x1, y1, x2, y2 in [(0, -10, 20, -10), (20, -10, 20, 10),
                           (20, 10, 0, 10), (0, 10, 0, -10)]:
        parts.append(f'\t(gr_line (start {x1} {y1}) (end {x2} {y2}) '
                     f'(stroke (width 0.1) (type default)) (layer "Edge.Cuts") '
                     f'(uuid "{uuid.uuid4()}"))')
    parts.append(')')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(parts) + '\n')
    if netclass:
        cls = {'clearance': 0.15, 'via_diameter': 0.5, 'via_drill': 0.3}
        with open(os.path.splitext(path)[0] + '.kicad_pro', 'w',
                  encoding='utf-8') as f:
            json.dump({'net_settings': {
                'classes': [dict(cls, name='Default', track_width=0.15),
                            dict(cls, name='Power', track_width=0.4)],
                'netclass_patterns': [{'netclass': 'Power',
                                       'pattern': 'SIG'}]}}, f)
    return evidence(path, 'the synthetic board')


def _route(td, name, extra, **kw):
    b = _board(os.path.join(td, name + '.kicad_pcb'), **kw)
    return PY + [tool('route.py'), b, os.path.join(td, name + '_out.kicad_pcb'),
                 '--nets', 'SIG', '--layers', 'F.Cu', '--strict-sizes'] + extra


IMP = ['--impedance', '50', '--track-width', '0.15', '--clearance', '0.15']


def t_impedance_neck_fails_strict_sizes(td):
    check(_route(td, 'imp', IMP), code=3,
          refuse='impedance copper shipped under width')
    print("  ok  --impedance 50 necked through the pinch: exit 3, row named")


def t_class_width_neck_fails_strict_sizes(td):
    check(_route(td, 'cls', [], netclass=True), code=3,
          refuse='class-width copper shipped under width')
    print("  ok  a 0.4 mm class necked through the pinch: exit 3, row named")


def t_controls(td):
    for name, kw, what in (
            ('open', {'wall_gap': 3.0}, 'no pinch, no neck'),
            ('stub', {'wall_gap': 3.0, 'stub': True},
             'a 0.1 mm SIG stub on the INPUT board')):
        r = check(_route(td, name, IMP, **kw), accept=True)
        out = r.stdout + r.stderr
        assert LINE not in out, f"{what}: printed a Design rules line\n" \
                                f"{out[-800:]}"
        print(f"  ok  control ({what}): exit 0, no Design rules line")


def t_repair_planes_power_taps(td):
    sf = os.path.join(td, 'sf.kicad_pcb')
    subprocess.run(PY + [tool('copy_board.py'),
                         os.path.join(ROOT, 'kicad_files',
                                      'splitflap_driver.kicad_pcb'), sf],
                   check=True, capture_output=True, cwd=ROOT)
    poured = os.path.join(td, 'poured.kicad_pcb')
    check(PY + [tool('route_planes.py'), sf, poured, '--nets', 'GND',
                '--plane-layers', 'B.Cu'], accept=True)
    evidence(poured, 'the poured board')
    r = check(PY + [tool('repair_planes.py'), poured,
                    os.path.join(td, 'repaired.kicad_pcb'), '--power-nets',
                    'GND', '--power-nets-widths', '1.0', '--strict-sizes'],
              code=3, refuse='power copper shipped under width')
    js = [l for l in r.stdout.splitlines() if l.startswith('JSON_SUMMARY: ')]
    assert js, r.stdout[-800:]
    rows = json.loads(js[-1].split(': ', 1)[1])['design_rules']['narrowed']
    sites = [x['site'] for x in rows]
    assert sites == ['power copper shipped under width'], sites
    print(f"  ok  repair_planes --power-nets: exit 3, one shipped row "
          f"({rows[0]['length_mm']} mm at {rows[0]['delivered']:g}), no "
          f"per-attempt plane tap rows")


def t_ledger_rule(td):
    import fab_tiers
    prev = fab_tiers.get_escalation_policy()
    try:
        fab_tiers.set_escalation_policy('fab')
        for _ in range(3):
            fab_tiers.note_narrowing(7, 'track_width', 1.0, 0.2, 'plane tap')
        fab_tiers.note_narrowing(8, 'via_diameter', 0.5, 0.3, 'net rescue')
        fab_tiers.replace_track_rows({7}, [], 'impedance copper shipped '
                                              'under width')
        rows = fab_tiers.escalation_summary()['narrowed']
        assert [(r['net'], r['kind']) for r in rows] == [(8, 'via_diameter')], \
            rows
        fab_tiers.replace_track_rows(
            {7}, [(7, 'N7', 1.0, 0.2, 3.5)], 'impedance copper shipped '
                                             'under width')
        rows = fab_tiers.escalation_summary()['narrowed']
        assert [(r['net'], r['site'], r['count']) for r in rows] == [
            (8, 'net rescue', 1),
            (7, 'impedance copper shipped under width', 1)], rows
    finally:
        fab_tiers.set_escalation_policy(*prev)
    print("  ok  per-attempt rows of a net that ships nothing narrow are "
          "dropped; one row per net that does")


CASES = [t_ledger_rule, t_impedance_neck_fails_strict_sizes,
         t_class_width_neck_fails_strict_sizes, t_controls,
         t_repair_planes_power_taps]


def main():
    want = set(sys.argv[1:])
    for fn in CASES:
        if want and fn.__name__ not in want:
            continue
        td = tempfile.mkdtemp(prefix='twide_')
        try:
            print(fn.__name__)
            fn(td)
        finally:
            shutil.rmtree(td, ignore_errors=True)
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
