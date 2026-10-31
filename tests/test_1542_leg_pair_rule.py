#!/usr/bin/env python3
"""A pair routed at a rule binding P against N below the class clearance
(#1542) keeps that rule in its single-ended pieces too.

#1542 lets route_diff lay a pair at its class diff_pair_gap where that is
below the class clearance (KiCad's implicit "netclass '<class>' (diff pair)"
rule). The coupled middle honoured it; three single-ended pieces of the same
pair did not:

  1. the hybrid's terminal legs (`_route_hybrid_leg`) stamped the partner's
     copper at the partner's CLASS clearance, so the leg's own middle end --
     laid at the rule's gap, inside that halo -- had every neighbour cell
     blocked and no leg could attach ("terminal legs could not attach to the
     coupled middle"), and the pair fell to another layer or single-ended;
  2. the legs and the #444 seam re-asks are grid A* routes, so where P and N
     run alongside they sit at the nearest grid pitch, never at the rule's gap
     (a 0.1 mm rule, 0.13 mm tracks, 0.1 mm grid: a 0.17 gap on an axis, 0.153
     at 45 degrees), and read UNCOUPLED against 1.5x the 0.1 gap
     (`pull_to_pair_gap` lays them at the gap);
  3. the corpus grader staged the board minimum clearance up to the graded
     clearance, and the board minimum floors KiCad's implicit pair rule, so
     every coupled run laid at the class gap graded as a clearance item
     (`kicad_drc_compare._staged_copy`).

Each check runs once with the rule and once without it (the negative
control): without a rule the change must be inert.

    python3 tests/test_1542_leg_pair_rule.py
"""
import json
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'rust_router', 'tests', os.path.join('tests', 'stress')):
    sys.path.insert(0, os.path.join(ROOT, _d))

from kicad_parser import BoardInfo, Net                            # noqa: E402
from routing_config import GridRouteConfig, GridCoord              # noqa: E402
from obstacle_map import (build_base_obstacle_map,                  # noqa: E402
                          add_segments_list_as_obstacles,
                          remove_segments_list_from_obstacles)
from synth import make_pcb, make_seg                               # noqa: E402
from run_utils import evidence                                     # noqa: E402
import diff_pair_routing as dpr                                    # noqa: E402

RUN_ALL_TIMEOUT = 180
CLR = 0.2       # class clearance
RULE = 0.1      # the pair rule (class diff_pair_gap)
W = 0.13
P, N = 1, 2

fails = []


def check(name, ok, detail=''):
    print(('PASS' if ok else 'FAIL') + f': {name}' + (f' -- {detail}' if detail else ''))
    if not ok:
        fails.append(name)


def edge_gap(s, others):
    return dpr._seg_to_seglist_min_edge(s.start_x, s.start_y, s.end_x, s.end_y,
                                        s.width, s.layer, others)


def config(with_rule):
    cfg = GridRouteConfig(layers=['F.Cu'], track_width=W, clearance=CLR,
                          grid_step=0.1, diff_pair_gap=RULE)
    if with_rule:
        cfg.coupled_pair_clearances = {frozenset((P, N)): {
            'track': {'F.Cu': RULE}, 'layer': {'F.Cu': RULE},
            'pad': {'F.Cu': RULE}, 'stack': RULE}}
    return cfg


def board(segments):
    return make_pcb(
        segments=list(segments), nets={P: Net(P, '/D_P'), N: Net(N, '/D_N')},
        board_info=BoardInfo(layers={0: 'F.Cu'}, board_bounds=(5.0, 5.0, 15.0, 15.0),
                             copper_layers=['F.Cu']))


# ---------------------------------------------------------------- 1
def t_leg():
    """N's coupled middle runs y = 10.0 from x = 10.1; P's middle end sits
    above it at the rule's pitch (0.23), so on the 0.1 grid its cell is
    (10.1, 10.2). The leg comes from (10.1, 11.0)."""
    print("1. a leg attaches to its middle end laid at the pair rule's gap")
    n_mid = make_seg(10.1, 10.0, 14.0, 10.0, width=W, net_id=N)
    mid_pt = (10.1, 10.0 + W + RULE, 0)
    term = (10.1, 11.0, 0)
    coord = GridCoord(0.1)
    for with_rule in (False, True):
        cfg = config(with_rule)
        pcb = board([n_mid])
        obs = build_base_obstacle_map(pcb, cfg, nets_to_route=[P, N])
        tag = 'with the rule' if with_rule else 'negative control, no rule'
        # precondition: the class halo closes every cell around the end
        add_segments_list_as_obstacles(obs, [n_mid], cfg)
        mgx, mgy = coord.to_grid(mid_pt[0], mid_pt[1])
        around = [obs.is_blocked(mgx + dx, mgy + dy, 0)
                  for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)]
        remove_segments_list_from_obstacles(obs, [n_mid], cfg)
        if not with_rule:
            check(f"precondition ({tag}): the class halo blocks all 8 cells around the end",
                  all(around), f"{sum(around)}/8")
        segs, _v, _it, _path = dpr._route_hybrid_leg(
            pcb, P, cfg, obs, ['F.Cu'], coord, mid_pt, term, [n_mid], [], [])
        if with_rule:
            check(f"{tag}: the leg attaches", segs is not None)
            if segs:
                worst = min(edge_gap(s, [n_mid]) for s in segs)
                check(f"{tag}: every leg segment clears N by the rule", worst >= RULE - 1e-6,
                      f"{worst:.4f} mm")
        else:
            # Without a rule the partner is priced at its class clearance, as
            # before #1542 was fixed here: the end is boxed in.
            check(f"{tag}: no leg (the class halo, unchanged)", segs is None)


# ---------------------------------------------------------------- 2
def t_pull():
    """Two single-ended runs side by side at a grid pitch: P straight at
    y = 10.0, N above it at a 0.17 edge gap, then diagonal at a 0.153 gap."""
    print("2. single-ended runs alongside are laid at the rule's gap")
    p = [make_seg(10.0, 10.0, 12.0, 10.0, width=W, net_id=P)]
    n_y = 10.0 + W + 0.17
    n = [make_seg(9.6, n_y + 0.4, 10.0, n_y, width=W, net_id=N),
         make_seg(10.0, n_y, 12.0, n_y, width=W, net_id=N),
         make_seg(12.0, n_y, 12.4, n_y + 0.4, width=W, net_id=N)]
    cand = p + n
    pcb = board([])
    out0 = dpr.pull_to_pair_gap(cand, pcb, P, N, N, config(False))
    check("negative control, no rule: the identity (the same list back)", out0 is cand)
    out = dpr.pull_to_pair_gap(cand, pcb, P, N, N, config(True))
    run = [s for s in out if s.net_id == N and abs(s.start_y - s.end_y) < 1e-9]
    gap = edge_gap(run[0], p) if run else None
    check("with the rule: the straight run is laid at the rule's gap",
          gap is not None and RULE - 1e-6 <= gap <= RULE + 0.002, f"{gap}")
    ends = sorted((round(s.start_x, 3), round(s.start_y, 3)) for s in out if s.net_id == N)
    check("...its free ends stay where they were",
          (9.6, round(n_y + 0.4, 3)) in ends,
          f"{ends}")
    check("...and P is not moved", [s for s in out if s.net_id == P] == p)

    # 45-degree pair four grid cells apart: lines y = x and y = x + 0.4 are
    # 0.283 apart, a 0.153 edge gap
    pd = [make_seg(10.0, 10.0, 12.0, 12.0, width=W, net_id=P)]
    nd = [make_seg(9.8, 10.4, 10.0, 10.4, width=W, net_id=N),
          make_seg(10.0, 10.4, 11.6, 12.0, width=W, net_id=N),
          make_seg(11.6, 12.0, 11.6, 12.4, width=W, net_id=N)]
    g0 = edge_gap(nd[1], pd)
    out = dpr.pull_to_pair_gap(pd + nd, pcb, P, N, N, config(True))
    diag = [s for s in out if s.net_id == N
            and abs(abs(s.end_x - s.start_x) - abs(s.end_y - s.start_y)) < 1e-6
            and abs(s.end_x - s.start_x) > 1.0]
    g1 = edge_gap(diag[0], pd) if diag else None
    check(f"with the rule: a 45-degree run at {g0:.3f} is laid at the rule's gap",
          g1 is not None and RULE - 1e-6 <= g1 <= RULE + 0.002, f"{g1}")


# ---------------------------------------------------------------- 3
def t_staging():
    """The grader's staged project: the board minimum is capped at a class
    diff_pair_gap below its class clearance (KiCad floors that pair rule at the
    board minimum)."""
    print("3. the corpus grader keeps KiCad's pair rule in force")
    import kicad_drc_compare as kdc
    src = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
    for gap, want, tag in ((RULE, RULE, 'class gap 0.1 under clearance 0.2'),
                           (0.25, 0.1375, 'negative control, class gap 0.25 (no pair rule)')):
        d = tempfile.mkdtemp(prefix='t1542leg_')
        try:
            pcb = os.path.join(d, 'b.kicad_pcb')
            shutil.copy(src, pcb)
            pro = {'board': {'design_settings': {'rules': {'min_clearance': 0.0}}},
                   'net_settings': {'classes': [
                       {'name': 'Default', 'clearance': 0.1375, 'diff_pair_gap': 0.25},
                       {'name': 'HS', 'clearance': CLR, 'diff_pair_gap': gap}]}}
            with open(os.path.join(d, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
                json.dump(pro, fh)
            sd, sb = kdc._staged_copy(evidence(pcb, 'staged input'), 0.1375)
            with open(evidence(os.path.splitext(sb)[0] + '.kicad_pro', 'staged project'),
                      encoding='utf-8') as fh:
                mc = json.load(fh)['board']['design_settings']['rules']['min_clearance']
            check(f"{tag}: staged board minimum {want:g}", abs(mc - want) < 1e-9, f"{mc}")
            shutil.rmtree(sd, ignore_errors=True)
        finally:
            shutil.rmtree(d, ignore_errors=True)


if __name__ == '__main__':
    t_leg()
    t_pull()
    t_staging()
    if fails:
        print(f"\nFAILED: {fails}")
        sys.exit(1)
    print("\nALL PASSED")
