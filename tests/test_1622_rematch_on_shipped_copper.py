#!/usr/bin/env python3
"""#1622: a pair the post-route cleanup moves out of its tolerance is matched
again on the copper the cleanup left.

route_diff matches P to N BEFORE its post-route cleanup, and the cleanup then
changes lengths with no regard to the match: an octolinear re-bend that
cleared a graze took 1.378 mm out of a member the matcher had brought to
0.089 mm, and the pair shipped at 1.466 mm against a 0.127 mm tolerance. The
regrade reported it honestly, and nothing went back to the matcher.

`length_matching.rematch_intra_pairs_on_shipped_copper` reads each pair's
shipped skew (KiCad's measure, `diff_pair_skew`) and meanders a pair over its
tolerance once more, keeping the new copper only when that skew goes down.

Checks:
  1. A matched pair whose member a "cleanup" then shortened is meandered
     again: the shipped skew falls under the tolerance, the board and the
     result hold the same copper, and `rematch` records it.
  2. A pair already inside its tolerance is not touched.
  3. A re-match that does not lower the shipped skew is undone: the board
     and the result hold exactly the copper they held before.
  4. A pair with no single coupled result (members routed apart) is left
     alone, as the matcher leaves it.

    python3 tests/test_1622_rematch_on_shipped_copper.py
"""
import contextlib
import io
import os
import sys
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import length_matching                                            # noqa: E402
from kicad_parser import BoardInfo, Net, Pad, PCBData, Segment    # noqa: E402
from length_matching import rematch_intra_pairs_on_shipped_copper  # noqa: E402
from net_queries import diff_pair_skew                            # noqa: E402
from routing_config import GridRouteConfig                        # noqa: E402

failures = []
P, N = 1, 2
TOL = 0.127


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def seg(net, pts):
    return [Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1], width=0.1,
                    layer='F.Cu', net_id=net) for a, b in zip(pts, pts[1:])]


def pad(net, x, y, ref):
    return Pad(pad_number='1', net_id=net, net_name=('X_P' if net == P else 'X_N'),
               shape='rect', global_x=x, global_y=y, local_x=0.0, local_y=0.0,
               size_x=0.1, size_y=0.1, layers=['F.Cu'], drill=0.0, pad_type='smd',
               component_ref=ref)


def case(n_end):
    """P straight 20 mm; N straight from (0, 0.3) to (n_end, 0.3), as a
    cleanup that straight-lined the jog the matcher put there leaves it."""
    p = seg(P, [(0, 0), (20, 0)])
    n = seg(N, [(0, 0.3), (n_end, 0.3)])
    info = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'],
                     board_bounds=(-10.0, -10.0, 30.0, 10.0), stackup=[])
    pcb = PCBData(board_info=info, nets={P: Net(net_id=P, name='X_P'),
                                         N: Net(net_id=N, name='X_N')},
                  footprints={}, segments=p + n, vias=[],
                  pads_by_net={P: [pad(P, 0, 0, 'U1'), pad(P, 20, 0, 'U2')],
                               N: [pad(N, 0, 0.3, 'U1'), pad(N, n_end, 0.3, 'U2')]})
    result = {'new_segments': p + n, 'new_vias': [], 'p_net_id': P, 'n_net_id': N}
    pair = SimpleNamespace(p_net_id=P, n_net_id=N, p_net_name='X_P', n_net_name='X_N')
    rec = {'pair': 'X', 'p_net': 'X_P', 'n_net': 'X_N', 'status': 'matched',
           'reason': None, 'delta_mm': 0.05, 'delta_before_mm': 2.0,
           'tolerance_mm': TOL}
    return pcb, result, pair, rec


CFG = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.1, diff_pair_gap=0.1,
                      clearance=0.1, grid_step=0.05, length_match_tolerance=0.1)


def run(pcb, result, pair, rec):
    with contextlib.redirect_stdout(io.StringIO()):
        return rematch_intra_pairs_on_shipped_copper(
            [rec], [('X', pair)], {P: result, N: result}, CFG, pcb)


def skew(pcb):
    return diff_pair_skew(pcb, P, N)['skew_mm']


def same_copper(pcb, result):
    return ({id(s) for s in pcb.segments} == {id(s) for s in result['new_segments']}
            and len(pcb.segments) == len(result['new_segments']))


def main():
    # 1. N comes out of the "cleanup" 1.5 mm short.
    pcb, result, pair, rec = case(18.5)
    before = skew(pcb)
    check('fixture: the cleanup left the pair over its tolerance',
          before is not None and before > TOL, f'{before}')
    kept = run(pcb, result, pair, rec)
    after = skew(pcb)
    check('the pair is matched again on the shipped copper',
          kept == 1 and after is not None and after <= TOL,
          f'{before:.4f} -> {after}')
    check('the board and the result hold the same copper', same_copper(pcb, result))
    check('the record says what the re-match did',
          rec.get('rematch', {}).get('kept') is True
          and rec['rematch']['skew_before_mm'] == round(before, 4)
          and rec['rematch']['skew_after_mm'] == round(after, 4), str(rec.get('rematch')))

    # 2. A pair inside its tolerance is not touched.
    pcb, result, pair, rec = case(19.95)
    ids = [id(s) for s in pcb.segments]
    kept = run(pcb, result, pair, rec)
    check('a pair inside its tolerance is not touched',
          kept == 0 and 'rematch' not in rec and [id(s) for s in pcb.segments] == ids)

    # 3. A re-match that lowers nothing is undone, exactly.
    pcb, result, pair, rec = case(18.5)
    seg_ids, res_ids = [id(s) for s in pcb.segments], [id(s) for s in result['new_segments']]
    real = length_matching.apply_intra_pair_length_matching

    def worse(res, cfg, pcb_data, p_net_id=None, n_net_id=None, status=None):
        # lengthens the LONGER leg: the shipped skew goes up
        res['new_segments'] = [s for s in res['new_segments'] if s.net_id != P] \
            + seg(P, [(0, 0), (10, 0), (10, -1), (11, -1), (11, 0), (20, 0)])
        if status is not None:
            status.update(status='matched', reason=None, delta_mm=0.0)
        return res
    length_matching.apply_intra_pair_length_matching = worse
    try:
        kept = run(pcb, result, pair, rec)
    finally:
        length_matching.apply_intra_pair_length_matching = real
    check('a re-match that does not lower the shipped skew is undone',
          kept == 0 and rec.get('rematch', {}).get('kept') is False
          and [id(s) for s in pcb.segments] == seg_ids
          and [id(s) for s in result['new_segments']] == res_ids,
          str(rec.get('rematch')))
    check('...and the record keeps the matcher status it had',
          rec['status'] == 'matched' and rec['delta_mm'] == 0.05)

    # 4. Members routed apart: no single result to meander.
    pcb, result, pair, rec = case(18.5)
    ids = [id(s) for s in pcb.segments]
    with contextlib.redirect_stdout(io.StringIO()):
        kept = rematch_intra_pairs_on_shipped_copper(
            [rec], [('X', pair)], {P: result, N: dict(result)}, CFG, pcb)
    check('a pair with no single coupled result is left alone',
          kept == 0 and 'rematch' not in rec and [id(s) for s in pcb.segments] == ids)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
