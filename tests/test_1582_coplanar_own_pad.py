#!/usr/bin/env python3
"""#1582: a coplanar line that lands on a pad wider than itself is not a broken
promise.

check_impedance measures a CPW line's gap from the TRACK's half-width. Over
the net's own pad the conductor is the pad, and just past it the pour still
keeps its clearance from the pad, so every line landing on a pad wider than
the track read a gap off its declaration there -- and one 0.15 mm sample is
over board_score's 0.01 mm gate. Three correct 50 ohm lines (routed and
poured as docs/api-impedance.md says) blocked DONE as two broken promises,
and check_impedance exited 1 with no line saying why: its audit printed for
the call-level --coplanar-gap only, not for gaps declared in the project.

Gated here, on four 30 mm lines (2.0 x 1.5 launch pads, 0402-sized pads,
pads narrower than the track, 1.0 x 2.4 pads wider across the line than the
audit probes), their gap declared in the project only:

  A. Poured as declared: no net's promise is broken, the length at the own
     pads is reported apart, and the line on the narrow pad has none.
  B. No pour beside the lines (B.Cu only): every line reads 100% no ground.
  C. A gap in the pour mid-line still counts -- the exemption is pad-local.
  D. A pour beside the line but not beside its pads counts at the pads.
  E. check_impedance prints the audit for a project-declared gap and exits 0
     on A, 1 on B naming the nets; board_score --impedance-nets reads
     blocking 0 on A and counts every net on B.

    python3 tests/test_1582_coplanar_own_pad.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)
sys.path.insert(0, TESTS_DIR)
for _p in ('py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from run_utils import evidence                                 # noqa: E402

failures = []

# The fourth pad is so wide across the line that the pour beside it sits past
# the audit's probe: it must be probed past the pad, not read as no ground.
NETS = (('/RF_LAUNCH', 2.0, 1.5), ('/RF_0402', 0.54, 0.64), ('/RF_NARROW', 0.2, 0.2),
        ('/RF_WIDE', 1.0, 2.4))
WIDTH = 0.27        # what route.py --impedance 50 --coplanar-gap 0.2 solves here


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def zone(layer, rects, uid):
    out = []
    for i, (x0, y0, x1, y1) in enumerate(rects):
        out.append(
            f' (zone (net 1) (net_name "GND") (layer "{layer}")'
            f' (uuid "00000000-0000-0000-0002-{uid:06d}{i:06d}") (hatch edge 0.5)'
            f' (connect_pads yes (clearance 0.2)) (min_thickness 0.1)'
            f' (filled_areas_thickness no) (fill yes (thermal_gap 0.2)'
            f' (thermal_bridge_width 0.2))'
            f' (polygon (pts (xy {x0} {y0}) (xy {x1} {y0}) (xy {x1} {y1}) (xy {x0} {y1}))))')
    return out


def build(path, front_rects):
    """The issue's board, routed: each line from pad centre to pad centre on
    F.Cu, GND poured on B.Cu over the whole board and on F.Cu over
    `front_rects`."""
    lines = [
        '(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")',
        ' (general (thickness 0.27)) (paper "A4")',
        ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))',
        ' (setup (stackup (layer "F.Cu" (type "copper") (thickness 0.035))',
        '   (layer "dielectric 1" (type "core") (thickness 0.2) (material "FR4")'
        ' (epsilon_r 4.5) (loss_tangent 0.02))',
        '   (layer "B.Cu" (type "copper") (thickness 0.035))) (pad_to_mask_clearance 0))',
        ' (net 0 "") (net 1 "GND")']
    lines += [f' (net {i + 2} "{n}")' for i, (n, _, _) in enumerate(NETS)]

    def fp(ref, x, y, net, name, w, h):
        return (f' (footprint "T:{ref}" (layer "F.Cu") (at {x} {y})'
                f' (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") (hide yes)'
                f' (effects (font (size 1 1) (thickness 0.15))))'
                f' (attr smd) (pad "1" smd rect (at 0 0) (size {w} {h})'
                f' (layers "F.Cu") (net {net} "{name}")))')
    for i, (name, w, h) in enumerate(NETS):
        y = 106 + 6 * i
        lines += [fp(f'J{2 * i + 1}', 105, y, i + 2, name, w, h),
                  fp(f'J{2 * i + 2}', 135, y, i + 2, name, w, h),
                  f' (segment (start 105 {y}) (end 135 {y}) (width {WIDTH})'
                  f' (layer "F.Cu") (net {i + 2})'
                  f' (uuid "00000000-0000-0000-0000-{i + 1:012d}"))']
    lines += [fp('G1', 120, 102, 1, 'GND', 1, 1), fp('G2', 120, 128, 1, 'GND', 1, 1)]
    lines.append(' (gr_rect (start 100 100) (end 140 130) (stroke (width 0.05)'
                 ' (type solid)) (fill none) (layer "Edge.Cuts")'
                 ' (uuid "00000000-0000-0000-0001-000000000001"))')
    lines += zone('F.Cu', front_rects, 1)
    lines += zone('B.Cu', [(100.2, 100.2, 139.8, 129.8)], 2)
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(lines) + '\n)\n')
    spec = {'ohms': 50.0, 'differential': False, 'coplanar_gap': 0.2, 'applied': True}
    with open(path[:-len('.kicad_pcb')] + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rules': {'min_clearance': 0.2}}},
                   'meta': {'filename': os.path.basename(path), 'version': 1},
                   'kicad_routing_tools': {'net_impedance':
                                           {n: dict(spec) for n, _, _ in NETS}}},
                  fh, indent=1)
    return path


def measure(path):
    """check_impedance's own reading, the way its main() makes it: every net
    at the gap its project declares."""
    from kicad_parser import parse_kicad_pcb
    from check_impedance import analyze_impedance
    pcb = parse_kicad_pcb(path)
    gaps = {nid: 0.2 for nid, n in pcb.nets.items() if n.name.startswith('/RF_')}
    rep = analyze_impedance(pcb, net_declared_gaps=gaps)
    return {n.net_name: n for n in rep.nets}


def bad(n):
    return n.length_no_ground_mm + n.length_gap_wrong_mm


def main():
    tmp = tempfile.mkdtemp(prefix='t1582_')
    try:
        full = (100.2, 100.2, 139.8, 129.8)
        boards = {
            'A': build(os.path.join(tmp, 'a.kicad_pcb'), [full]),
            'B': build(os.path.join(tmp, 'b.kicad_pcb'), []),
            'C': build(os.path.join(tmp, 'c.kicad_pcb'),
                       [(100.2, 100.2, 118, 129.8), (122, 100.2, 139.8, 129.8)]),
            'D': build(os.path.join(tmp, 'd.kicad_pcb'), [(109, 100.2, 131, 129.8)]),
        }

        print('A. poured as declared')
        a = measure(boards['A'])
        check('four lines measured', sorted(a) == sorted(n for n, _, _ in NETS),
              str(sorted(a)))
        check('no broken promise on any line',
              not any(n.promise_broken for n in a.values()),
              {k: round(bad(n), 3) for k, n in a.items()}.__repr__())
        check('the length at the wide pads is reported apart, not lost',
              a['/RF_LAUNCH'].length_at_own_pad_mm >= 2.0
              and a['/RF_0402'].length_at_own_pad_mm > 0.4,
              f"{a['/RF_LAUNCH'].length_at_own_pad_mm:.2f} / "
              f"{a['/RF_0402'].length_at_own_pad_mm:.2f}")
        check('...and is no more than the pad plus its clearance step',
              a['/RF_LAUNCH'].length_at_own_pad_mm <= 2 * (1.0 + 0.2 + 0.15) + 0.3
              and a['/RF_0402'].length_at_own_pad_mm <= 2 * (0.27 + 0.2 + 0.15) + 0.3,
              f"{a['/RF_LAUNCH'].length_at_own_pad_mm:.2f} / "
              f"{a['/RF_0402'].length_at_own_pad_mm:.2f}")
        check('a pad wider than the probe is probed past, not read as no ground',
              a['/RF_WIDE'].length_no_ground_mm == 0.0
              and a['/RF_WIDE'].length_at_own_pad_mm > 0.4,
              f"no ground {a['/RF_WIDE'].length_no_ground_mm:.2f}, "
              f"at pad {a['/RF_WIDE'].length_at_own_pad_mm:.2f}")
        check('a pad narrower than the track exempts nothing (its gap is right)',
              a['/RF_NARROW'].length_at_own_pad_mm == 0.0,
              f"{a['/RF_NARROW'].length_at_own_pad_mm:.2f}")

        print('B. no pour beside the lines')
        b = measure(boards['B'])
        check('every line reads 100% no ground',
              all(abs(n.length_no_ground_mm - n.outer_length_mm) < 1e-6
                  and n.outer_length_mm > 29 and n.promise_broken
                  for n in b.values()),
              str({k: (round(n.length_no_ground_mm, 2), round(n.outer_length_mm, 2))
                   for k, n in b.items()}))

        print('C. a gap in the pour mid-line')
        c = measure(boards['C'])
        check('every line is broken by it, the narrow one too',
              all(n.promise_broken and bad(n) >= 3.0 for n in c.values()),
              str({k: round(bad(n), 2) for k, n in c.items()}))

        print('D. a pour beside the line but not beside its pads')
        d = measure(boards['D'])
        check('the launch line counts at its pads: no ground there',
              d['/RF_LAUNCH'].promise_broken
              and d['/RF_LAUNCH'].length_no_ground_mm >= 2.0,
              f"no ground {d['/RF_LAUNCH'].length_no_ground_mm:.2f}, "
              f"at pad {d['/RF_LAUNCH'].length_at_own_pad_mm:.2f}")

        print('E. the CLIs')
        for arm, code in (('A', 0), ('B', 1)):
            js = os.path.join(tmp, f'imp_{arm}.json')
            r = subprocess.run([sys.executable, '-X', 'utf8',
                                os.path.join(ROOT, 'py_tools', 'check_impedance.py'),
                                boards[arm], '--json', js],
                               capture_output=True, text=True, encoding='utf-8',
                               errors='replace', cwd=ROOT, timeout=900)
            out = r.stdout + r.stderr
            check(f'{arm}: check_impedance exits {code}', r.returncode == code,
                  f'exit {r.returncode}: {out[-300:]}')
            check(f'{arm}: the audit prints for the project-declared gap',
                  'Coplanar declaration audit (declared gap 0.2mm' in out, out[-600:])
            doc = json.load(open(evidence(js, 'check_impedance --json')))
            names = doc['totals'].get('nets_broken_promise')
            if arm == 'A':
                check('A: the audit says the pads are not counted',
                      'at its own pad:' in out and 'achieved its gap' in out, out[-600:])
                check('A: no net in nets_broken_promise', names == [], str(names))
            else:
                check('B: the audit names every net it fails',
                      all(n in out for n, _, _ in NETS)
                      and 'NOT achieved' in out, out[-600:])
                check('B: every net in nets_broken_promise',
                      names == sorted(n for n, _, _ in NETS), str(names))
        for arm, want in (('A', 0), ('B', 4)):
            js = os.path.join(tmp, f'score_{arm}.json')
            subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_tools', 'board_score.py'),
                            boards[arm], '--impedance-nets', '/RF_*', '--quiet',
                            '--json', js], capture_output=True, text=True,
                           encoding='utf-8', errors='replace', cwd=ROOT, timeout=900)
            imp = json.load(open(evidence(js, 'board_score --json')))['components']['impedance']
            check(f'{arm}: board_score --impedance-nets counts {want}',
                  imp.get('count') == want
                  and len(imp.get('nets_broken_promise') or ()) == want,
                  json.dumps(imp)[:300])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
