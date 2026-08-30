#!/usr/bin/env python3
"""Issue #766: --diff-pair-intra-match length-matched only SOME pairs, silently.

    python3 tests/test_766_intra_pair_every_pair.py

The old intra-pair loop walked `routed_results` and required each result to
carry `is_diff_pair` + `p_net_id`. Those stamps are applied by the 2-point
coupled constructor and the multipoint merge -- but NOT by the direct-hybrid
escape (`_route_direct_hybrid`, "HYBRID ESCAPE: direct coupled middle +
point-to-point terminal legs"), whose result is a bare four-key dict:

    {'new_segments', 'new_vias', 'iterations', 'path_length'}

So a pair routed that way was skipped WITHOUT PRINTING ANYTHING, while the run
still counted it routed. Measured on haasoscope_pro_max_test (10 LVDS pairs,
4 routed, all four via the hybrid escape): the intra-pair section printed zero
pair sections and four pairs shipped at 1.19-7.25mm P/N skew. The reporter's
6-pair MIPI/CSI board is the same shape -- one true coupled pair matched at
0.18mm, four hybrid-routed pairs shipped at 1.69-4.00mm.

Two directions are pinned here, because only the pair of them shows the
mechanism rather than the symptom:

  POSITIVE  a hybrid-shaped result (no is_diff_pair, no p_net_id) IS matched
            when the driver supplies the pair's net ids, and EVERY pair in the
            run gets exactly one record -- including ones with no result.
  NEGATIVE  the same result, matched WITHOUT the caller-supplied ids (what the
            old code did), refuses with reason 'no-pair-net-ids'. This is the
            change detector: delete the id plumbing and this arm goes green
            while the positive arm goes red.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))

from kicad_parser import Segment, Net, PCBData, BoardInfo
from routing_config import GridRouteConfig, DiffPairNet
from length_matching import (apply_intra_pair_length_matching,
                             run_intra_pair_matching)

P_ID, N_ID = 101, 102
Q_ID, R_ID = 201, 202


def _seg(x1, y1, x2, y2, net, w=0.15, layer='F.Cu'):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=w, layer=layer, net_id=net)


def _pcb(segments):
    return PCBData(
        footprints={},
        nets={P_ID: Net(P_ID, '/PAIR_P'), N_ID: Net(N_ID, '/PAIR_N'),
              Q_ID: Net(Q_ID, '/OTHER_P'), R_ID: Net(R_ID, '/OTHER_N')},
        segments=segments, vias=[],
        board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu']),
        pads_by_net={})


def run():
    fails = []

    def check(name, cond, detail=""):
        if not cond:
            fails.append(name)
        print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))

    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.15,
                          clearance=0.15, diff_pair_gap=0.15)
    cfg.length_match_tolerance = 0.15
    cfg.diff_pair_intra_match = True

    # P runs 30mm, N runs 26mm on the same layer, well apart so meanders fit.
    p_segs = [_seg(10.0, 10.0, 40.0, 10.0, P_ID)]
    n_segs = [_seg(10.0, 15.0, 36.0, 15.0, N_ID)]
    # A second pair with copper on the board but NO in-run result at all.
    q_segs = [_seg(10.0, 30.0, 25.0, 30.0, Q_ID)]
    r_segs = [_seg(10.0, 35.0, 21.0, 35.0, R_ID)]

    pcb = _pcb(p_segs + n_segs + q_segs + r_segs)

    # Exactly the shape _route_direct_hybrid returns -- no is_diff_pair, no
    # p_net_id, no route_length.
    hybrid = {'new_segments': list(p_segs) + list(n_segs), 'new_vias': [],
              'iterations': 1234, 'path_length': 7}

    pair = DiffPairNet(base_name='/PAIR', p_net_id=P_ID, n_net_id=N_ID,
                       p_net_name='/PAIR_P', n_net_name='/PAIR_N')
    other = DiffPairNet(base_name='/OTHER', p_net_id=Q_ID, n_net_id=R_ID,
                        p_net_name='/OTHER_P', n_net_name='/OTHER_N')

    # --- NEGATIVE CONTROL: the old call shape refuses ----------------------
    ctl = {}
    apply_intra_pair_length_matching(dict(hybrid), cfg, pcb, status=ctl)
    check("negative control: hybrid result without caller ids is not matched",
          ctl.get('status') == 'unmatched' and ctl.get('reason') == 'no-pair-net-ids',
          f"status={ctl.get('status')!r} reason={ctl.get('reason')!r}")

    # --- POSITIVE: driven from the pair list -------------------------------
    routed_results = {P_ID: hybrid, N_ID: hybrid}
    reports = run_intra_pair_matching(
        [('/PAIR', pair), ('/OTHER', other)], routed_results, cfg, pcb)

    check("every pair in the run gets exactly one record",
          len(reports) == 2 and [r['pair'] for r in reports] == ['/OTHER', '/PAIR'],
          str([r['pair'] for r in reports]))

    rec = {r['pair']: r for r in reports}
    hy = rec.get('/PAIR', {})
    check("hybrid-routed pair is matched, not skipped",
          hy.get('status') in ('matched', 'improved'),
          f"status={hy.get('status')!r} reason={hy.get('reason')!r}")
    check("hybrid-routed pair's skew is reduced",
          (hy.get('delta_before_mm') is not None
           and hy.get('delta_mm') is not None
           and hy['delta_mm'] < hy['delta_before_mm']),
          f"{hy.get('delta_before_mm')}mm -> {hy.get('delta_mm')}mm")
    check("meanders were actually emitted onto the shorter member",
          len(hybrid['new_segments']) > len(p_segs) + len(n_segs),
          f"{len(hybrid['new_segments'])} segs")

    # A pair with no in-run result is REPORTED with its measured board skew,
    # never omitted -- that silence is what shipped 4.00mm skew as success.
    ot = rec.get('/OTHER', {})
    check("unroutable-here pair is reported, not omitted",
          ot.get('status') == 'not-matchable'
          and ot.get('reason') == 'pair-not-routed-this-run',
          f"status={ot.get('status')!r} reason={ot.get('reason')!r}")
    check("its skew is still measured off the board",
          ot.get('delta_mm') is not None and abs(ot['delta_mm'] - 4.0) < 0.05,
          f"delta={ot.get('delta_mm')}mm (expected ~4.0)")

    # --- the AC-coupled skip stays a SKIP, with its reason ------------------
    reports2 = run_intra_pair_matching(
        [('/PAIR', pair)], {P_ID: dict(hybrid), N_ID: dict(hybrid)}, cfg, pcb,
        skip_p_net_ids={P_ID})
    check("ac-coupled member pair is skipped with a named reason",
          len(reports2) == 1 and reports2[0]['status'] == 'skipped'
          and reports2[0]['reason'] == 'ac-coupled-xnet',
          str(reports2))

    # --- the group pass must not call this-run copper "an earlier step" -----
    # The reporter chained five extra invocations because the group matcher
    # told them to "route the whole group in one step to meander it" -- on a
    # run that WAS one step. A hybrid-routed member has an in-run result with
    # no `route_length`, so it is board-seeded; flagging it `routed_this_run`
    # is what lets the warning stop giving that advice.
    from length_matching import _seed_group_members_from_board
    names = ['/PAIR_P', '/PAIR_N', '/OTHER_P']
    seeded = _seed_group_members_from_board(
        names, have_names=set(), pcb_data=pcb,
        in_run_names={'/PAIR_P', '/PAIR_N'})
    check("hybrid member seeded from board is flagged routed-this-run",
          seeded.get('/PAIR_P', {}).get('routed_this_run') is True
          and seeded.get('/PAIR_N', {}).get('routed_this_run') is True,
          str({k: v.get('routed_this_run') for k, v in seeded.items()}))
    check("a genuine earlier-step member is NOT flagged routed-this-run",
          seeded.get('/OTHER_P', {}).get('routed_this_run') is False,
          str(seeded.get('/OTHER_P', {}).get('routed_this_run')))

    # --- a hybrid escape must not silently claim coupled terminals ---------
    # The hybrid route IS fully routed (pads connect, DRC clean) so it keeps
    # its credit -- but 'coupled' means "both members routed coupled" and a
    # hybrid's TERMINAL legs are point-to-point single-ended copper. Disclose,
    # do not reclassify: demoting to 'partial' would drop it out of
    # routed_diff_pairs and imply follow-up work that does not exist.
    from diff_pair_custody import build_pair_reports

    class _St:
        def __init__(self, rr):
            self.routed_results = rr
            self.diff_pair_single_ended_nets = {}
            self.pair_diagnostics = {}

    hyb_res = {'new_segments': list(p_segs) + list(n_segs), 'new_vias': [],
               'iterations': 9, 'path_length': 3, 'hybrid_escape': True}
    true_res = {'new_segments': list(p_segs) + list(n_segs), 'new_vias': [],
                'iterations': 9, 'path_length': 3, 'is_diff_pair': True,
                'p_net_id': P_ID, 'n_net_id': N_ID}
    audit = {'/PAIR': {'p': True, 'n': True}}

    hy_rep = build_pair_reports(_St({P_ID: hyb_res, N_ID: hyb_res}),
                                [('/PAIR', pair)], audit)[0]
    check("hybrid escape is disclosed in its pair report",
          hy_rep.get('escape') == 'hybrid'
          and hy_rep.get('coupled_terminals') is False,
          str({k: hy_rep.get(k) for k in ('outcome', 'escape', 'coupled_terminals')}))
    check("hybrid escape KEEPS its routed credit (outcome unchanged)",
          hy_rep.get('outcome') == 'coupled', str(hy_rep.get('outcome')))

    tr_rep = build_pair_reports(_St({P_ID: true_res, N_ID: true_res}),
                                [('/PAIR', pair)], audit)[0]
    check("negative control: a genuinely coupled pair carries no hybrid marker",
          'escape' not in tr_rep and 'coupled_terminals' not in tr_rep
          and tr_rep.get('outcome') == 'coupled',
          str({k: tr_rep.get(k) for k in ('outcome', 'escape', 'coupled_terminals')}))

    print()
    if fails:
        print(f"FAILED ({len(fails)}): " + ", ".join(fails))
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(run())
