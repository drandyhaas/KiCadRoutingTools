#!/usr/bin/env python3
"""#1623: the intra-pair matcher's measure agrees with KiCad's skew.

`net_signal_length` (what route_diff --diff-pair-intra-match matches on) read
the walk of its copper graph, where a track node inside a pad joined it at no
cost and a via was charged a flat share of its whole span. KiCad's skew DRC
(`kicad_net_length_nm`, which `diff_pair_skew` and check_drc read) cuts a run
ending at a pad's anchor at the pad's outline, charges any other in-pad
copper whole, and charges a via the stackup between the layers its tracks
use. So a pair KiCad read at 0.014 mm read 1.11 mm (a 1.1 mm launch piece
inside a 2.4 mm pad was free), and the matcher meandered it out to 1.10 mm.

The walk now only SELECTS the copper (the pad-to-pad path plus the pieces
lying inside its pads; dead ends trimmed on a larger net), and that copper
is measured by KiCad's model.

Checks, one cause per pair, 30 mm between 2.4 x 0.3 mm pads (N one straight
track unless the row says otherwise; a 0.1/0.6/0.1/0.1/0.1 mm stack):
  1. the matcher's skew equals `diff_pair_skew` for a straight control, a
     joint inside the pad, a track starting behind the anchor inside the pad,
     an in-pad launch piece, and P and N on inner layers of different depth;
  2. a dead-end branch, which the post-route cleanup trims, is still left out
     (`diff_pair_skew` charges it on copper that keeps it);
  3. `apply_intra_pair_length_matching` on the pair with a joint inside its
     pad (which the walk read 0.9 mm apart) reads it within tolerance and
     adds no meander.

    python3 tests/test_1623_signal_length_kicad_measure.py
"""
import contextlib
import io
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb                          # noqa: E402
from length_matching import apply_intra_pair_length_matching      # noqa: E402
from net_queries import diff_pair_skew, net_signal_length          # noqa: E402
from routing_config import GridRouteConfig                         # noqa: E402

failures = []
ROWS = ['CTRL', 'PADJOINT', 'PADOFF', 'LAUNCH', 'VIAHEIGHT', 'BRANCH']
LAYERS = [('In1.Cu', .1), ('In2.Cu', .6), ('In3.Cu', .1), ('In4.Cu', .1), ('B.Cu', .1)]


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def build(path):
    nid = {f'/{r}_{s}': i * 2 + (s == 'N') + 1 for i, r in enumerate(ROWS) for s in 'PN'}

    def fp(ref, x, y, net):
        return (f'  (footprint "T:{ref}" (layer "F.Cu") (at {x} {y})\n'
                f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") (hide yes)'
                f' (effects (font (size 1 1) (thickness 0.15))))\n    (attr smd)\n'
                f'    (pad "1" smd rect (at 0 0) (size 2.4 0.3) (layers "F.Cu") '
                f'(net {nid[net]} "{net}"))\n  )\n')

    def seg(x1, y1, x2, y2, net, la='F.Cu'):
        return (f'  (segment (start {x1} {y1}) (end {x2} {y2}) (width 0.15) '
                f'(layer "{la}") (net {nid[net]}))\n')

    def via(x, y, net):
        return (f'  (via (at {x} {y}) (size 0.5) (drill 0.2) (layers "F.Cu" "B.Cu") '
                f'(net {nid[net]}))\n')

    def viapath(y, net, la):
        return (seg(105, y, 115, y, net) + via(115, y, net) + seg(115, y, 125, y, net, la)
                + via(125, y, net) + seg(125, y, 135, y, net))

    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n (layers (0 "F.Cu" signal) '
         + ''.join(f'({2 if la == "B.Cu" else 2 + 2 * (i + 1)} "{la}" signal) '
                   for i, (la, _t) in enumerate(LAYERS))
         + '(25 "Edge.Cuts" user))\n (setup (stackup (layer "F.Cu" (type "copper") '
           '(thickness 0.035))\n')
    b += ''.join(f'   (layer "dielectric {i + 1}" (type "core") (thickness {t}) '
                 f'(material "FR4") (epsilon_r 4.5) (loss_tangent 0.02))\n'
                 f'   (layer "{la}" (type "copper") (thickness 0.035))\n'
                 for i, (la, t) in enumerate(LAYERS))
    b += '  ) (pad_to_mask_clearance 0))\n (net 0 "")\n' + ''.join(
        f' (net {i} "{n}")\n' for n, i in nid.items())
    for k, r in enumerate(ROWS):
        y, P, N = 105 + 3 * k, f'/{r}_P', f'/{r}_N'
        b += (fp(f'J{k}P', 105, y, P) + fp(f'K{k}P', 135, y, P)
              + fp(f'J{k}N', 105, y + 1, N) + fp(f'K{k}N', 135, y + 1, N))
        b += (viapath(y + 1, N, 'In2.Cu') if r == 'VIAHEIGHT'
              else seg(105, y + 1, 135, y + 1, N))
        b += {'CTRL': seg(105, y, 135, y, P),
              # the same straight line, with a joint 0.9 mm in, inside the pad
              'PADJOINT': seg(105, y, 105.9, y, P) + seg(105.9, y, 135, y, P),
              # starts inside the pad, 0.9 mm behind the anchor
              'PADOFF': seg(104.1, y, 135, y, P),
              # starts 0.1 mm off the anchor and runs 1 mm along the pad to a
              # joint, then out: an in-pad launch piece (route_diff laid one)
              'LAUNCH': seg(105.1, y, 106.1, y, P) + seg(106.1, y, 135, y, P),
              # P on In1.Cu, N on In2.Cu: an uneven stack
              'VIAHEIGHT': viapath(y, P, 'In1.Cu'),
              # a 0.6 mm dangling branch
              'BRANCH': seg(105, y, 135, y, P) + seg(120, y, 120, y + .6, P)}[r]
    for x1, y1, x2, y2 in ((100, 100, 140, 100), (140, 100, 140, 130),
                           (140, 130, 100, 130), (100, 130, 100, 100)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    with open(path, 'w') as fh:
        fh.write(b + ')\n')


def main():
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, 'rows.kicad_pcb')
        build(path)
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            pcb = parse_kicad_pcb(path)
    ids = {n.name: i for i, n in pcb.nets.items()}

    def skews(row):
        p, n = ids[f'/{row}_P'], ids[f'/{row}_N']
        lp, ln = net_signal_length(pcb, p), net_signal_length(pcb, n)
        return (None if lp is None or ln is None else abs(lp - ln),
                diff_pair_skew(pcb, p, n)['skew_mm'])

    for row, want in (('CTRL', 0.0), ('PADJOINT', 0.0), ('PADOFF', 0.9),
                      ('LAUNCH', 0.1), ('VIAHEIGHT', None)):
        mine, kicad = skews(row)
        check(f'{row}: the matcher reads the skew KiCad reads',
              mine is not None and kicad is not None and abs(mine - kicad) < 1e-4
              and (want is None or abs(kicad - want) < 1e-4),
              f'matcher {mine}, diff_pair_skew {kicad}')
    mine, kicad = skews('VIAHEIGHT')
    check('VIAHEIGHT: the inner layers are charged at their real depth',
          kicad is not None and kicad > 1.0, f'{kicad}')
    mine, kicad = skews('BRANCH')
    check('BRANCH: a dead end the cleanup trims is left out of the matcher\'s read',
          mine is not None and abs(mine) < 1e-6 and abs(kicad - 0.6) < 1e-4,
          f'matcher {mine}, diff_pair_skew {kicad}')

    # the matcher on a pair KiCad already has within tolerance
    p, n = ids['/PADJOINT_P'], ids['/PADJOINT_N']
    segs = [s for s in pcb.segments if s.net_id in (p, n)]
    result = {'new_segments': list(segs), 'new_vias': []}
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.15, diff_pair_gap=0.15,
                          clearance=0.15, grid_step=0.05, length_match_tolerance=0.1)
    status = {}
    with contextlib.redirect_stdout(io.StringIO()):
        apply_intra_pair_length_matching(result, cfg, pcb, p_net_id=p, n_net_id=n,
                                         status=status)
    check('a pair KiCad reads within tolerance gets no meander',
          status.get('status') == 'within-tolerance'
          and [id(s) for s in result['new_segments']] == [id(s) for s in segs],
          str(status))

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
