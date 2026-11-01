#!/usr/bin/env python3
"""#1597: the pre-existing-blocker hint says a rip leaves the blockers' pads.

`preexisting_blocker_hint` (#301/#103) names the pre-existing nets whose
copper walls a failed route and offers `--rip-existing-nets <them>`. On the
issue's board /CAGED's pad is ringed by four signal pins 0.4 mm away, each
with a via and a track: the hint named their nets, and two more all-nets laps
and the named rip each ended exactly where they began, because what walls the
pad is the four PADS, which no rip removes. `check_reachability --pad`, which
sets the routing aside, says CAGED.

The hint keeps the retry command (it is right whenever the copper is the
wall) and now says what the rip cannot do: their pads stay, and
check_reachability on the failed pad tells which case it is.

Checks:
  1. route /CAGED alone: it fails, and the hint names the four nets' retry
     command (the precondition: this is the hint #1597 is about).
  2. that hint says the pads stay and names check_reachability --pad.
  3. evidence the pointer is the right one: check_reachability on the input
     board's U1.1 says CAGED (exit 1), with the routing set aside.

    python3 tests/test_1597_rip_hint_names_the_pads.py
"""
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)

from run_utils import check as run_check, evidence, tool        # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def board():
    """The issue's board, less its /OVER control: /CAGED's U1.1 ringed by the
    pins of /S1../S4 at 0.4 mm, each pin carrying a via and a B.Cu track;
    a /TP pad under U1.1 on B.Cu; GND planes on both inner layers;
    /CAGED's other pad 3 mm away."""
    cx, cy, a = 120.0, 115.0, 0.40

    def fp(ref, x, y, pads, layer='F.Cu'):
        p = ''.join(f'    (pad "{n}" smd {sh} (at {px} {py}) (size {s} {s}) '
                    f'(layers "{layer}") (net {nid} "{nm}"))\n'
                    for n, sh, px, py, s, nid, nm in pads)
        return (f'  (footprint "T:{ref}" (layer "{layer}") (at {x} {y})\n'
                f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") '
                f'(hide yes) (effects (font (size 1 1) (thickness 0.15))))\n'
                f'    (attr smd)\n{p}  )\n')

    def zone(layer):
        return (f'  (zone (net 1) (net_name "GND") (layer "{layer}") (hatch edge 0.5) '
                f'(connect_pads yes (clearance 0.1))\n'
                f'    (min_thickness 0.1) (filled_areas_thickness no) (fill yes '
                f'(thermal_gap 0.2) (thermal_bridge_width 0.2))\n'
                f'    (polygon (pts (xy 100.5 100.5) (xy 139.5 100.5) (xy 139.5 129.5) '
                f'(xy 100.5 129.5))))\n')

    names = ['GND', '/S1', '/S2', '/S3', '/S4', '/CAGED', '/TP']
    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (6 "In2.Cu" signal) '
         '(2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
         ' (setup (pad_to_mask_clearance 0))\n (net 0 "")\n')
    b += ''.join(f' (net {i + 1} "{n}")\n' for i, n in enumerate(names))
    b += fp('U1', cx, cy, [('1', 'rect', 0, 0, .15, 6, '/CAGED'),
                           ('2', 'circle', a, 0, .35, 2, '/S1'),
                           ('3', 'circle', -a, 0, .35, 3, '/S2'),
                           ('4', 'circle', 0, a, .35, 4, '/S3'),
                           ('5', 'circle', 0, -a, .35, 5, '/S4')])
    for i, (x, y, ex, ey, nid, nm) in enumerate([(a, 0, 8, 0, 2, '/S1'),
                                                 (-a, 0, -8, 0, 3, '/S2'),
                                                 (0, a, 0, 8, 4, '/S3'),
                                                 (0, -a, 0, -8, 5, '/S4')]):
        b += (f'  (via (at {cx + x} {cy + y}) (size 0.35) (drill 0.2) '
              f'(layers "F.Cu" "B.Cu") (net {nid}))\n'
              f'  (segment (start {cx + x} {cy + y}) (end {cx + ex} {cy + ey}) '
              f'(width 0.1) (layer "B.Cu") (net {nid}))\n')
        b += fp(f'P{i + 1}', cx + ex, cy + ey, [('1', 'rect', 0, 0, .4, nid, nm)],
                layer='B.Cu')
    b += fp('TP1', cx, cy, [('1', 'rect', 0, 0, .15, 7, '/TP')], layer='B.Cu')
    # /CAGED's other pad, inside check_reachability's default +/-4 mm view
    # (a far one makes it widen the view and take minutes)
    b += fp('Y1', cx + 3, cy + 3, [('1', 'rect', 0, 0, .5, 6, '/CAGED')])
    b += fp('G1', 104, 104, [('1', 'rect', 0, 0, 1, 1, 'GND')])
    b += fp('G2', 136, 104, [('1', 'rect', 0, 0, 1, 1, 'GND')])
    b += zone('In1.Cu') + zone('In2.Cu')
    for x1, y1, x2, y2 in ((100, 100, 140, 100), (140, 100, 140, 130),
                           (140, 130, 100, 130), (100, 130, 100, 100)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    return b + ')\n'


def main():
    tmp = tempfile.mkdtemp(prefix='t1597_')
    try:
        inp = os.path.join(tmp, 'in.kicad_pcb')
        out = os.path.join(tmp, 'out.kicad_pcb')
        with open(inp, 'w', encoding='utf-8') as fh:
            fh.write(board())
        evidence(inp, 'synthetic input board')

        print("1. /CAGED fails, and the hint offers the rip")
        env = dict(os.environ, KICAD_PLANE_FINALIZE='0')
        r = run_check([sys.executable, '-X', 'utf8', tool('route.py'), inp, out,
                       '--nets', '/CAGED', '--clearance-ceiling', '0.1',
                       '--track-width', '0.1', '--via-size', '0.35',
                       '--via-drill', '0.2', '--layer-costs', '1', '-1', '-1', '1'],
                      accept=True, env=env)
        evidence(out, 'routed output')
        con = r.stdout + r.stderr
        check("precondition: /CAGED failed",
              '"failed_single": ["/CAGED"]' in con)
        hints = [ln.strip() for ln in con.splitlines()
                 if 'Hint: the blocking copper' in ln]
        offer = [h for h in hints if 'Retry with --rip-existing-nets' in h]
        check("precondition: the hint offers the four nets' rip",
              bool(offer) and all("'/S1'" in h and "'/S4'" in h for h in offer),
              (hints or [''])[0][:200])

        print("2. it says what the rip leaves")
        check("the pads stay", bool(offer) and "their pads stay" in offer[0],
              (offer or [''])[0][-400:])
        check("and it names check_reachability on the failed pad",
              bool(offer) and 'check_reachability.py <board> --pad <REF.PAD>'
              in offer[0])

        print("3. the instrument it names says CAGED here")
        cr = run_check([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_tools', 'check_reachability.py'),
                        inp, '--pad', 'U1.1', '--track', '0.09', '--via', '0.25',
                        # 0.02 mm raster: the cage's widest gap misses the
                        # 0.27 mm a track needs by 0.054 mm, and the default
                        # 0.01 costs minutes on a loaded box
                        '--clearance', '0.09', '--step', '0.02'],
                       refuse='CAGED for any track width', code=1)
        check("CAGED with the routing set aside",
              'routing set aside' in cr.stdout + cr.stderr)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
