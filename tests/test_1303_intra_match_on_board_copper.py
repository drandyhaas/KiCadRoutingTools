#!/usr/bin/env python3
"""#1303 phase (b)2: intra-pair matching measures P and N on the board's copper.

The matcher summed its routing result's own segments plus the stub lengths the
coupled router records. A hybrid escape, or a #444 seam re-ask, returns a
result with no stub lengths, so on imx6ulz DQS0 it read P 12.00 < N 13.08,
meandered P, and shipped P 21.33 > N 16.82: skew 4.51 -> 5.20 mm. And on a
multipoint pair (CLK0) it rewrote only the merged bookkeeping dict, so the
writer emitted the leg's original straight run beside the meander: the bump
was shorted out and KICAD_BOARD_LEDGER reported the net diverged.

Now `apply_intra_pair_length_matching` reads each leg's length off the board
(`net_queries.net_signal_length`: the pad-to-pad path of a 2-pad net, the
dead-end-trimmed copper of a larger one) and splices a multipoint meander
into the leg dict that held the replaced copper (`_splice_into_legs`).

    python3 tests/test_1303_intra_match_on_board_copper.py
"""
import contextlib
import io
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import BoardInfo, Net, Pad, PCBData, Segment  # noqa: E402
from length_matching import (_splice_into_legs,                  # noqa: E402
                             apply_intra_pair_length_matching)
from net_queries import net_signal_length                        # noqa: E402
from routing_config import GridRouteConfig                       # noqa: E402
from routing_utils import segment_length                         # noqa: E402

failures = []
P, N = 1, 2


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def seg(net, pts, layer='F.Cu'):
    return [Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1], width=0.1,
                    layer=layer, net_id=net) for a, b in zip(pts, pts[1:])]


def pad(net, x, y, ref='U1'):
    return Pad(pad_number='1', net_id=net, net_name=('X_P' if net == P else 'X_N'),
               shape='rect', global_x=x, global_y=y, local_x=0.0, local_y=0.0,
               size_x=0.1, size_y=0.1, layers=['F.Cu'], drill=0.0, pad_type='smd',
               component_ref=ref)


def board(segments, pads):
    info = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'],
                     board_bounds=(-10.0, -10.0, 40.0, 40.0), stackup=[])
    return PCBData(board_info=info, nets={P: Net(net_id=P, name='X_P'),
                                          N: Net(net_id=N, name='X_N')},
                   footprints={}, segments=list(segments), vias=[], pads_by_net=pads)


def length(segs):
    return sum(segment_length(s) for s in segs)


def signal_length_unit():
    # 2 pads, a 3 mm dead-end spur and a loop: the pad-to-pad path is 10 mm.
    net = (seg(P, [(0, 0), (10, 0)]) + seg(P, [(5, 0), (5, 3)])
           + seg(P, [(2, 0), (2, 1), (8, 1), (8, 0)]))
    pcb = board(net, {P: [pad(P, 0, 0), pad(P, 10, 0)]})
    check('a 2-pad net reads its pad-to-pad path (spur and loop are what the '
          'cleanup trims)', abs(net_signal_length(pcb, P) - 10.0) < 1e-6,
          str(net_signal_length(pcb, P)))
    # 3 pads in a T with a dead-end spur: the tree is 10 + 4 = 14 mm.
    net = (seg(P, [(0, 0), (10, 0)]) + seg(P, [(5, 0), (5, 4)])
           + seg(P, [(10, 0), (12, 0)]))
    pcb = board(net, {P: [pad(P, 0, 0), pad(P, 5, 4), pad(P, 10, 0)]})
    check('a 3-pad net reads its copper with dead-end branches trimmed',
          abs(net_signal_length(pcb, P) - 14.0) < 1e-6, str(net_signal_length(pcb, P)))
    pcb = board(seg(P, [(0, 0), (4, 0)]), {P: [pad(P, 0, 0), pad(P, 10, 0)]})
    check('pads not joined by track copper read None, never a stub length',
          net_signal_length(pcb, P) is None)


def wrong_leg_case():
    """The result says P is shorter; the board says N is (P carries a 4 mm
    fanout stub the result knows nothing about, as after a seam re-ask)."""
    stub_p = seg(P, [(2, -4), (2, 0)])
    res_p = seg(P, [(2, 0), (18, 0)])
    res_n = seg(N, [(2, 0.2), (10, 0.2), (10.5, 0.7), (11, 0.2), (18, 0.2)])
    pads = {P: [pad(P, 2, -4), pad(P, 18, 0, 'U2')],
            N: [pad(N, 2, 0.2), pad(N, 18, 0.2, 'U2')]}
    pcb = board(stub_p + res_p + res_n, pads)
    result = {'new_segments': res_p + res_n, 'new_vias': [], 'hybrid_escape': True}
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.1, diff_pair_gap=0.1,
                          clearance=0.1, grid_step=0.05, length_match_tolerance=0.1)
    status = {}
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        apply_intra_pair_length_matching(result, cfg, pcb, p_net_id=P, n_net_id=N,
                                         status=status)
    log = out.getvalue()
    new_p = [s for s in result['new_segments'] if s.net_id == P]
    new_n = [s for s in result['new_segments'] if s.net_id == N]
    check('the result alone reads P as the shorter leg (the old basis)',
          length(res_p) < length(res_n))
    check('the board reads N as shorter, and the matcher says which it used',
          'board copper P=20.000mm, N=16.414mm' in log, log.strip()[:300])
    check('the meander goes on N, the leg the board has shorter',
          length(new_p) == length(res_p) and length(new_n) > length(res_n) + 1.0,
          f'P {length(new_p):.3f} N {length(new_n):.3f}')
    check("...and brings the board's P/N within reach of tolerance",
          status.get('status') in ('matched', 'improved')
          and status.get('delta_before_mm') == round(20.0 - 16.4142, 4),
          str(status))


def splice_unit():
    """A multipoint pair: the write list carries the LEG dicts, the merged
    dict only bookkeeping. A meander must land in the leg."""
    l1 = {'new_segments': seg(P, [(0, 0), (5, 0)]) + seg(N, [(0, 0.2), (5, 0.2)])}
    l2 = {'new_segments': seg(P, [(5, 0), (9, 0)]) + seg(N, [(5, 0.2), (9, 0.2)])}
    merged = {'is_multipoint': True, 'leg_results': [l1, l2],
              'new_segments': l1['new_segments'] + l2['new_segments']}
    old_p = [s for s in merged['new_segments'] if s.net_id == P]
    straight = old_p[1]
    meander = seg(P, [(5, 0), (6, 0), (6, -1), (7, -1), (7, 0), (9, 0)])
    new_p = [old_p[0]] + meander
    all_before = list(merged['new_segments'])
    merged['new_segments'] = [s for s in all_before if s.net_id != P] + new_p
    _splice_into_legs(merged, old_p, new_p, all_before)
    written = [s for lr in (l1, l2) for s in lr['new_segments']]
    check('the leg that held the replaced run takes the meander',
          straight not in l2['new_segments'] and all(s in l2['new_segments'] for s in meander)
          and l1['new_segments'][0] is old_p[0])
    check('what the writer emits (the legs) is what the board gets (the merged list)',
          {id(s) for s in written} == {id(s) for s in merged['new_segments']}
          and len(written) == len(merged['new_segments']))


def main():
    signal_length_unit()
    wrong_leg_case()
    splice_unit()
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
