#!/usr/bin/env python3
"""#1629: net_pads_joined joins copper where it touches, as connectivity does.

`net_queries.net_pads_joined` (the gate on every skew and length-group grade)
joined a pad only to a track node inside the pad's AXIS BOX, a via only by its
centre, and never one pad to another. Connected copper then read "not
joined": check_drc printed "not measured (pads not joined by track copper)"
and board_score counted nothing, while check_connected passed the nets and
KiCad graded their skew out of range -- and the board read DONE.

Now a track joins a pad where its round end reaches the pad's exact outline,
a via joins a pad its barrel overlaps, and two same-net pads whose copper
touches are one terminal (check_connected's own rules). A pair that still
reads unjoined while connectivity passes it (its pads meet through a pour)
is listed in board_score's `ungraded`.

Checks (one pair per shape, P 3 mm shorter than N):
  1. two overlapping same-net pads with the track on one, a via whose barrel
     overlaps the pad with its centre outside, a track whose cap reaches the
     pad's edge, and a track ending in a 45-degree pad's copper outside its
     axis box: joined, and the skew is measured;
  2. the controls (track to the pad centre, a dangling via, a path through a
     pad) still join;
  3. copper that does NOT touch -- a cap 0.05 mm short of the edge, a pad
     0.1 mm from the one the track lands on, a barrel 0.05 mm from the
     pad -- does not join (kicad-cli 10.0.6 reports exactly these three
     rows unconnected, and every other row connected);
  4. check_drc grades the skew of the touching shapes under a .kicad_dru
     skew rule (none "not measured");
  5. board_score's `ungraded` names `diff_pairs.unjoined_pairs` for a
     connected pair nothing measured, and only while the component grades.

    python3 tests/test_1629_pads_joined_by_contact.py
"""
import contextlib
import io
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb                 # noqa: E402
from net_queries import diff_pair_skew, net_pads_joined  # noqa: E402
from run_utils import tool                               # noqa: E402

failures = []
TOUCHING = ['STACK', 'VIAEDGE', 'CAP', 'TILT']
CONTROLS = ['CTRL', 'DANGLE', 'THRU']
APART = ['CAPSHORT', 'STACKGAP', 'VIAGAP']


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def build(out, rows):
    nets = [f'/{m}_{s}' for m in rows for s in 'PN']
    nid = {n: i + 1 for i, n in enumerate(nets)}

    def pad(num, x, y, n, w=.6, h=.6, rot=0):
        return (f'    (pad "{num}" smd rect (at {x} {y} {rot}) (size {w} {h}) '
                f'(layers "F.Cu") (net {nid[n]} "{n}"))\n')

    def fp(ref, x, y, *pads):
        return (f'  (footprint "T:{ref}" (layer "F.Cu") (at {x} {y})\n'
                f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") '
                f'(hide yes) (effects (font (size 1 1) (thickness 0.15))))\n'
                f'    (attr smd)\n' + ''.join(pads) + '  )\n')

    def seg(x1, y1, x2, y2, n, lay='F.Cu'):
        return (f'  (segment (start {x1} {y1}) (end {x2} {y2}) (width 0.2) '
                f'(layer "{lay}") (net {nid[n]}))\n')

    def via(x, y, n):
        return (f'  (via (at {x} {y}) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") '
                f'(net {nid[n]}))\n')

    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
         ' (setup (pad_to_mask_clearance 0))\n (net 0 "")\n'
         + ''.join(f' (net {i} "{n}")\n' for n, i in nid.items()))
    for r, m in enumerate(rows):
        y, Y, P, N, A, B = 105 + r * 5, 106.5 + r * 5, f'/{m}_P', f'/{m}_N', 105, 135
        b += (fp(f'A{r}N', A, Y, pad('1', 0, 0, N)) + fp(f'B{r}N', B + 3, Y, pad('1', 0, 0, N))
              + seg(A, Y, B + 3, Y, N))
        b += fp(f'A{r}P', A, y, pad('1', 0, 0, P))
        far = fp(f'B{r}P', B, y, pad('1', 0, 0, P))
        b += {
            'CTRL': far + seg(A, y, B, y, P),
            'DANGLE': far + seg(A, y, B, y, P) + seg(A + 3, y, A + 3, y - 1, P)
                      + via(A + 3, y - 1, P),
            'THRU': far + fp(f'C{r}P', 120, y, pad('1', 0, 0, P)) + seg(A, y, 120, y, P)
                    + seg(120, y, B, y, P),
            # two overlapping same-net pads, the track on one
            'STACK': fp(f'B{r}P', B, y, pad('1', 0, 0, P), pad('1', 0.5, 0, P))
                     + seg(A, y, B, y, P),
            # ...and two 0.1 mm apart: not one terminal
            'STACKGAP': fp(f'B{r}P', B, y, pad('1', 0, 0, P), pad('1', 0.7, 0, P))
                        + seg(A, y, B, y, P),
            # a barrel overlapping the pad 0.1 mm, its centre outside
            'VIAEDGE': far + seg(A, y, A + 2, y, P) + via(A + 2, y, P)
                       + seg(A + 2, y, B - 0.5, y, P, 'B.Cu') + via(B - 0.5, y, P),
            # ...and one 0.05 mm short of it
            'VIAGAP': far + seg(A, y, A + 2, y, P) + via(A + 2, y, P)
                      + seg(A + 2, y, B - 0.65, y, P, 'B.Cu') + via(B - 0.65, y, P),
            # the centreline stops 0.05 short of the pad edge, the cap overlaps it
            'CAP': far + seg(A, y, B - 0.35, y, P),
            # ...and stops 0.15 short: the 0.1 mm cap misses by 0.05
            'CAPSHORT': far + seg(A, y, B - 0.45, y, P),
            # ends in a 45-degree pad's copper, outside its axis box
            'TILT': fp(f'B{r}P', B, y, pad('1', 0, 0, P, 2, 0.5, 45))
                    + seg(A, y, A, y + 0.6, P) + seg(A, y + 0.6, B - 0.6, y + 0.6, P),
        }[m]
    for x1, y1, x2, y2 in ((100, 100, 145, 100), (145, 100, 145, 160),
                           (145, 160, 100, 160), (100, 160, 100, 100)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    with open(out, 'w') as fh:
        fh.write(b + ')\n')


def main():
    rows = TOUCHING + CONTROLS + APART
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'shapes.kicad_pcb')
        build(path, rows)
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            pcb = parse_kicad_pcb(path)
        ids = {n.name: i for i, n in pcb.nets.items()}
        for m in TOUCHING + CONTROLS:
            p, n = ids[f'/{m}_P'], ids[f'/{m}_N']
            sk = diff_pair_skew(pcb, p, n)['skew_mm']
            check(f'{m}: joined, and the skew is measured',
                  net_pads_joined(pcb, p) and sk is not None, f'skew {sk}')
        for m in ('STACK', 'CAP', 'TILT'):
            sk = diff_pair_skew(pcb, ids[f'/{m}_P'], ids[f'/{m}_N'])['skew_mm']
            want = 3.35 if m == 'CAP' else 3.0
            check(f'{m}: the skew is the one KiCad reports ({want} mm)',
                  sk is not None and abs(sk - want) < 1e-4, f'{sk}')
        for m in APART:
            check(f'{m}: copper that does not touch does not join',
                  not net_pads_joined(pcb, ids[f'/{m}_P']))

        # check_drc grades the touching shapes' skew under a rule
        only = os.path.join(tmp, 'touching.kicad_pcb')
        build(only, TOUCHING)
        with open(os.path.join(tmp, 'touching.kicad_dru'), 'w') as fh:
            fh.write('(version 1)\n(rule "pair skew"\n  (condition "A.inDiffPair(\'*\')")\n'
                     '  (constraint skew (max 0.1mm) (within_diff_pairs)))\n')
        with open(os.path.join(tmp, 'touching.kicad_pro'), 'w') as fh:
            fh.write('{"meta":{"version":2},"net_settings":{"classes":[{"name":"Default",'
                     '"clearance":0.1,"track_width":0.2,"via_diameter":0.6,'
                     '"via_drill":0.3}]}}\n')
        r = subprocess.run([sys.executable, '-B', tool('check_drc.py'), only,
                            '--clearance', '0.1'], capture_output=True, text=True,
                           cwd=ROOT)
        line = next((ln for ln in r.stdout.splitlines()
                     if ln.startswith('Diff-pair rules')), '')
        check('check_drc grades every touching pair\'s skew',
              f'skew: {len(TOUCHING)} group(s)/pair(s) graded' in line
              and 'not measured' not in line, line or (r.stdout + r.stderr)[-300:])

    # board_score: a connected pair nothing measured is listed ungraded
    from board_score import ungraded_entries
    uj = [{'p_net': '/X_P', 'n_net': '/X_N'}]
    got = ungraded_entries({'diff_pairs': {'ran': True, 'count': 0,
                                           'unjoined_connected': uj}})
    check('board_score lists a connected, unmeasured pair in ungraded',
          'diff_pairs.unjoined_pairs' in got, str(got))
    got = ungraded_entries({'diff_pairs': {'ran': True, 'count': 0,
                                           'unjoined_pairs': uj}})
    check('...not a pair connectivity already reports open',
          'diff_pairs.unjoined_pairs' not in got, str(got))
    got = ungraded_entries({'diff_pairs': {'ran': False, 'unjoined_connected': uj}})
    check('...and not twice when the whole component is ungraded',
          'diff_pairs.unjoined_pairs' not in got and 'diff_pairs' in got, str(got))

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
