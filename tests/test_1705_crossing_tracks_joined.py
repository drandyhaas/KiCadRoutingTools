#!/usr/bin/env python3
"""#1705: two same-net tracks that CROSS mid-span join, in net_pads_joined too.

`net_queries._net_track_graph` (the graph under `net_pads_joined`, and so
under `diff_pair_skew`, `net_signal_length`, check_drc's skew rule and
board_score's diff_pairs) cut a segment only where another segment's
ENDPOINT lay on it, or where it passed through a pad or via. Two same-layer
tracks crossing in an X, neither ending on the other, got no common node: a
pair joined only by such a crossing read "not joined", check_drc printed
"not measured (pads not joined by track copper)" and board_score counted no
pair, while check_connected passed the net and kicad-cli 10.0.6 graded its
skew out of range (actual -5.1680 mm, the number pinned below).

Now each segment is also cut where another segment of its layer crosses it
(check_connected's `segments_intersect` rule), so the crossing is a node.

Checks (P joined only by an X crossing, P 5.168 mm longer than N):
  1. net_pads_joined reads the crossing as a join, as check_connected does;
  2. diff_pair_skew is KiCad's 5.168 mm, the same as for the same copper
     split at the crossing;
  3. net_signal_length walks through the crossing (15.0 mm: the 1 mm and
     0.168 mm overshoots past it are dead ends);
  4. controls: the vertical track stopping 0.3 mm short of the horizontal,
     and the same X with the two tracks on different layers, still do not
     join;
  5. check_drc grades the pair's skew under a .kicad_dru skew rule (1 out
     of range, none not measured), and board_score blocks on it.

    python3 tests/test_1705_crossing_tracks_joined.py
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
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb                 # noqa: E402
from net_queries import (diff_pair_skew, net_pads_joined,  # noqa: E402
                         net_signal_length)
from check_connected import check_net_connectivity       # noqa: E402
from run_utils import evidence, tool                     # noqa: E402

failures = []
NETS = {'/CK_P': 1, '/CK_N': 2}


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def build(out, shape):
    def pad(ref, x, y, net):
        return (f'  (footprint "T:{ref}" (layer "F.Cu") (at {x} {y})\n'
                f'    (property "Reference" "{ref}" (at 0 -1 0) (layer "F.SilkS") '
                f'(hide yes) (effects (font (size 1 1) (thickness 0.15))))\n'
                f'    (attr smd)\n    (pad "1" smd circle (at 0 0) (size 0.4 0.4) '
                f'(layers "F.Cu" "B.Cu") (net {NETS[net]} "{net}"))\n  )\n')

    def seg(x1, y1, x2, y2, net, lay='F.Cu'):
        return (f'  (segment (start {x1} {y1}) (end {x2} {y2}) (width 0.14) '
                f'(layer "{lay}") (net {NETS[net]}))\n')

    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
         ' (setup (pad_to_mask_clearance 0))\n (net 0 "")\n'
         + ''.join(f' (net {i} "{n}")\n' for n, i in NETS.items()))
    b += (pad('J1', 105, 105, '/CK_P') + pad('J2', 115, 110, '/CK_P')
          + pad('J3', 105, 112, '/CK_N') + pad('J4', 116, 112, '/CK_N'))
    b += {
        # the horizontal runs 1 mm past x=115; the vertical crosses it and
        # ends 0.168 mm past it
        'cross': seg(105, 105, 116, 105, '/CK_P') + seg(115, 110, 115, 104.832, '/CK_P'),
        'split': (seg(105, 105, 115, 105, '/CK_P') + seg(115, 105, 116, 105, '/CK_P')
                  + seg(115, 110, 115, 105, '/CK_P') + seg(115, 105, 115, 104.832, '/CK_P')),
        'short': seg(105, 105, 116, 105, '/CK_P') + seg(115, 110, 115, 105.3, '/CK_P'),
        'layers': (seg(105, 105, 116, 105, '/CK_P')
                   + seg(115, 110, 115, 104.832, '/CK_P', 'B.Cu')),
    }[shape]
    b += seg(105, 112, 116, 112, '/CK_N')
    for x1, y1, x2, y2 in ((100, 100, 125, 100), (125, 100, 125, 118),
                           (125, 118, 100, 118), (100, 118, 100, 100)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    with open(out, 'w') as fh:
        fh.write(b + ')\n')
    stem = out[:-len('.kicad_pcb')]
    with open(stem + '.kicad_dru', 'w') as fh:
        fh.write('(version 1)\n(rule "skew"\n  (condition "A.inDiffPair(\'*\')")\n'
                 '  (constraint skew (max 0.127mm) (within_diff_pairs)))\n')
    with open(stem + '.kicad_pro', 'w') as fh:
        fh.write('{"meta":{"version":2},"net_settings":{"classes":[{"name":"Default",'
                 '"clearance":0.1,"track_width":0.14,"via_diameter":0.6,'
                 '"via_drill":0.3}]}}\n')


def load(path):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    ids = {n.name: i for i, n in pcb.nets.items()}
    return pcb, ids['/CK_P'], ids['/CK_N']


def main():
    with tempfile.TemporaryDirectory() as tmp:
        paths = {}
        for shape in ('cross', 'split', 'short', 'layers'):
            paths[shape] = os.path.join(tmp, f'{shape}.kicad_pcb')
            build(paths[shape], shape)
            evidence(paths[shape], f'the {shape} board')

        pcb, p, n = load(paths['cross'])
        conn = check_net_connectivity(p, [s for s in pcb.segments if s.net_id == p], [],
                                      pcb.pads_by_net[p])
        check('check_connected joins the X crossing (the reference)',
              conn.get('connected') is True, str(conn.get('connected')))
        check('net_pads_joined reads the X crossing as a join', net_pads_joined(pcb, p))
        sk = diff_pair_skew(pcb, p, n)['skew_mm']
        check('diff_pair_skew is the 5.168 mm kicad-cli reports',
              sk is not None and abs(sk - 5.168) < 1e-6, str(sk))
        sl = net_signal_length(pcb, p)
        check('net_signal_length walks through the crossing (15.0 mm)',
              sl is not None and abs(sl - 15.0) < 1e-6, str(sl))

        spcb, sp, sn = load(paths['split'])
        ssk = diff_pair_skew(spcb, sp, sn)['skew_mm']
        check('the same copper split at the crossing reads the same skew',
              ssk is not None and sk is not None and abs(ssk - sk) < 1e-9,
              f'{ssk} vs {sk}')

        for shape, why in (('short', 'a track stopping 0.3 mm short'),
                           ('layers', 'the X on two layers')):
            cpcb, cp, _cn = load(paths[shape])
            check(f'control: {why} does not join', not net_pads_joined(cpcb, cp))

        r = subprocess.run([sys.executable, '-B', tool('check_drc.py'), paths['cross'],
                            '--clearance', '0.1'], capture_output=True, text=True, cwd=ROOT)
        line = next((ln for ln in r.stdout.splitlines()
                     if ln.startswith('Diff-pair rules')), '')
        check('check_drc grades the crossing pair\'s skew out of range',
              'skew: 1 group(s)/pair(s) graded, 1 net(s) out of range' in line
              and 'not measured' not in line, line or (r.stdout + r.stderr)[-300:])

        r = subprocess.run([sys.executable, '-B', tool('board_score.py'), paths['cross']],
                           capture_output=True, text=True, cwd=ROOT)
        line = next((ln for ln in r.stdout.splitlines() if ln.startswith('BLOCKING=')), '')
        check('board_score blocks on the pair\'s skew',
              'diff_pairs=1' in line, line or (r.stdout + r.stderr)[-300:])

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
