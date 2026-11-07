#!/usr/bin/env python3
"""#1704: a member that runs past a corner and comes back along itself is
measured as KiCad measures it.

route_diff's intra-pair matcher read each member with
`net_queries.net_signal_length`, both before the post-route cleanup and in
#1622's re-match on the shipped copper. The pad-to-pad walk cut the
outbound segment where the return leg's far end lands back on it (a
T-junction 5 um off its line), and the shortest walk then left out the
overshoot and the return leg: N read 18.995 mm while all its copper, and
KiCad's net length, is 20.995 mm. The cleanup keeps that copper (by segment
endpoints it is a simple path), so the re-match read "already matched" on a
pair kicad-cli 10.0.6 grades `actual -2.0000 mm`.

Now the re-match reads the shipped copper with KiCad's measure
(`net_match_length`, which `diff_pair_skew` reads), and the walk charges a
loop that meets the pad-to-pad path at ONE node only: it is copper hanging
off one point of the path, not a shortcut the cleanup takes, unless every
segment in it is one the cleanup can drop whole.

Checks (P 18.995 mm; N the same plus a 1 mm overshoot and its 1 mm return):
  1. net_signal_length charges the doubled-back copper (20.995 mm), and the
     control (the same N without it) still reads 18.995 mm;
  2. route_diff's own post-route cleanup keeps the doubled-back copper (the
     precondition: what ships is what KiCad grades);
  3. the re-match on the shipped copper meanders P: the shipped skew falls
     inside the 0.127 mm tolerance and the record says matched;
  4. a closed loop of whole segments hanging off one node (prunable copper)
     is still left out, and a dead-end branch is still trimmed;
  5. the shipped measure on its own: on copper no cleanup will touch again,
     the matcher reads that same branch as KiCad does and matches it.

    python3 tests/test_1704_doubled_back_member.py
"""
import contextlib
import io
import os
import sys
import tempfile
import types

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import parse_kicad_pcb                    # noqa: E402
from net_queries import (diff_pair_skew, net_copper_length,  # noqa: E402
                         net_signal_length)
from routing_config import GridRouteConfig                  # noqa: E402
from cleanup_pipeline import run_post_route_cleanup         # noqa: E402
from length_matching import (apply_intra_pair_length_matching,  # noqa: E402
                             rematch_intra_pairs_on_shipped_copper,
                             regrade_intra_pair_reports)
from run_utils import evidence                              # noqa: E402

failures = []
NETS = {'/CK_P': 1, '/CK_N': 2}
TOL = 0.127


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
                f'(layers "F.Cu") (net {NETS[net]} "{net}"))\n  )\n')

    def seg(x1, y1, x2, y2, net):
        return (f'  (segment (start {x1} {y1}) (end {x2} {y2}) (width 0.15) '
                f'(layer "F.Cu") (net {NETS[net]}))\n')

    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
         ' (setup (pad_to_mask_clearance 0))\n (net 0 "")\n'
         + ''.join(f' (net {i} "{n}")\n' for n, i in NETS.items()))
    b += (pad('J1', 105, 105, '/CK_P') + pad('J2', 119, 109.995, '/CK_P')
          + pad('J3', 105, 106, '/CK_N') + pad('J4', 117, 113, '/CK_N'))
    b += seg(105, 105, 119, 105, '/CK_P') + seg(119, 105, 119, 109.995, '/CK_P')
    b += {
        # N runs 1 mm past its corner at x=117, turns back along itself and
        # ends 5 um off its own line, then leaves south from there
        'fold': (seg(105, 106, 118, 106, '/CK_N') + seg(118, 106, 117, 106.005, '/CK_N')
                 + seg(117, 106.005, 117, 113, '/CK_N')),
        'plain': seg(105, 106, 117, 106.005, '/CK_N') + seg(117, 106.005, 117, 113, '/CK_N'),
        # a closed square of whole segments hanging off the corner node
        'ring': (seg(105, 106, 117, 106, '/CK_N') + seg(117, 106, 117, 113, '/CK_N')
                 + seg(117, 106, 118, 106, '/CK_N') + seg(118, 106, 118, 107, '/CK_N')
                 + seg(118, 107, 117.5, 107, '/CK_N') + seg(117.5, 107, 117, 106, '/CK_N')),
        # a dead-end branch off the corner
        'branch': (seg(105, 106, 117, 106, '/CK_N') + seg(117, 106, 117, 113, '/CK_N')
                   + seg(117, 106, 119, 106, '/CK_N')),
    }[shape]
    for x1, y1, x2, y2 in ((100, 100, 125, 100), (125, 100, 125, 118),
                           (125, 118, 100, 118), (100, 118, 100, 100)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    with open(out, 'w') as fh:
        fh.write(b + ')\n')


def load(path):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    ids = {n.name: i for i, n in pcb.nets.items()}
    return pcb, ids['/CK_P'], ids['/CK_N']


def main():
    with tempfile.TemporaryDirectory() as tmp:
        paths = {}
        for shape in ('fold', 'plain', 'ring', 'branch'):
            paths[shape] = os.path.join(tmp, f'{shape}.kicad_pcb')
            build(paths[shape], shape)
            evidence(paths[shape], f'the {shape} board')

        pcb, p, n = load(paths['fold'])
        sl = net_signal_length(pcb, n)
        check('fold: net_signal_length charges the doubled-back copper (20.995 mm)',
              sl is not None and abs(sl - 20.995) < 1e-3,
              f'{sl}; all copper {net_copper_length(pcb, n):.4f}')
        cpcb, _cp, cn = load(paths['plain'])
        sl = net_signal_length(cpcb, cn)
        check('control: the same N without the fold reads 18.995 mm',
              sl is not None and abs(sl - 18.995) < 1e-3, f'{sl}')
        rpcb, _rp, rn = load(paths['ring'])
        sl = net_signal_length(rpcb, rn)
        check('a ring of whole segments off one node is still left out (19.0 mm)',
              sl is not None and abs(sl - 19.0) < 1e-3, f'{sl}')
        bpcb, _bp, bn = load(paths['branch'])
        sl = net_signal_length(bpcb, bn)
        check('a dead-end branch is still trimmed (19.0 mm)',
              sl is not None and abs(sl - 19.0) < 1e-3, f'{sl}')

        # route_diff's post-route cleanup (its own switches), then #1622's
        # re-match on what it left, as route_diff calls them
        cfg = GridRouteConfig(length_match_tolerance=TOL, track_width=0.15,
                              clearance=0.15, layers=['F.Cu', 'B.Cu'])
        res = {'new_segments': [s for s in pcb.segments if s.net_id in (p, n)],
               'new_vias': [], 'p_net_id': p, 'n_net_id': n}
        before = len(pcb.segments)
        with contextlib.redirect_stdout(io.StringIO()):
            run_post_route_cleanup([res], pcb, {p, n}, cfg, label='Diff-pair ',
                                   snap=False, phantom=False, neck=False)
        check('route_diff\'s cleanup keeps the doubled-back copper (5 of 5 segments)',
              len(pcb.segments) == before == 5
              and abs(net_copper_length(pcb, n) - 20.995) < 1e-3,
              f'{len(pcb.segments)} of {before}; N {net_copper_length(pcb, n):.4f}')
        sk = diff_pair_skew(pcb, p, n)['skew_mm']
        check('the shipped skew before the re-match is KiCad\'s 2.000 mm',
              sk is not None and abs(sk - 2.0) < 1e-3, f'{sk}')
        pair = types.SimpleNamespace(p_net_id=p, n_net_id=n)
        reps = [{'pair': 'CK', 'status': 'within-tolerance', 'reason': None,
                 'delta_mm': 0.0, 'tolerance_mm': TOL}]
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            rematch_intra_pairs_on_shipped_copper(reps, [('CK', pair)], {p: res, n: res},
                                                  cfg, pcb)
            regrade_intra_pair_reports(reps, None, pcb, [('CK', pair)], TOL)
        sk = diff_pair_skew(pcb, p, n)['skew_mm']
        r = reps[0]
        check('the re-match meanders P: the shipped skew is inside the tolerance',
              sk is not None and sk <= TOL and (r.get('rematch') or {}).get('kept'),
              f'skew {sk}; rematch {r.get("rematch")}')
        check('the record says matched',
              r.get('status') == 'matched' and r.get('reason') is None,
              f"{r.get('status')} {r.get('reason')} delta {r.get('delta_mm')}")
        if failures:
            print(out.getvalue())

        # the shipped measure on its own: copper no cleanup will touch again
        # (here a dead-end branch, which the estimate leaves out) is read as
        # KiCad reads it, so the 2 mm the branch adds is matched
        bpcb, bp, bn = load(paths['branch'])
        res = {'new_segments': [s for s in bpcb.segments if s.net_id in (bp, bn)],
               'new_vias': [], 'p_net_id': bp, 'n_net_id': bn}
        status = {}
        with contextlib.redirect_stdout(io.StringIO()):
            apply_intra_pair_length_matching(res, cfg, bpcb, p_net_id=bp, n_net_id=bn,
                                             status=status, shipped=True)
        check('shipped=True reads all the copper KiCad grades (delta 2.005 mm) '
              'and matches it',
              abs((status.get('delta_before_mm') or 0) - 2.005) < 1e-3
              and status.get('status') == 'matched', str(status))

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
