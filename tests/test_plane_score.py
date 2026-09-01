#!/usr/bin/env python3
"""Plane fragility as a placement score (--plane-score, #118 follow-up).

The candidate score was plane-blind: crossings/hpwl/inversions are signal
proxies, while a placement that splits the ground pour or squeezes it through
necks loses at the PLANE step -- and #424 already prices exact fill damage
router-side. Contract under test:

  * plane_score.plane_fragility_score pours the named nets on a scratch copy
    and reduces the KiCad fill to (islands, neck_sum) -- honest None when no
    KiCad python exists;
  * rank_key orders plane_islands BEFORE hpwl (a split pour is a measured
    loss, hpwl is a proxy) and plane_neck after it (tiebreak);
  * absent metrics leave the ranking exactly as before (flag off = inert).
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # placement split
from placement.portfolio import Candidate, rank_key  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _cand(index, crossings=10, hpwl=100.0, **extra):
    c = Candidate(index=index, strategy='jitter', board='b.kicad_pcb')
    c.metrics = dict({'crossings': crossings, 'inversions': 0, 'hpwl': hpwl},
                     **extra)
    return c


def test_parse_plane_specs():
    from plane_score import parse_plane_specs
    specs = parse_plane_specs(['GND', '3V3:F.Cu'], ['F.Cu', 'B.Cu'])
    assert specs == [('GND', 'B.Cu'), ('3V3', 'F.Cu')], specs
    print("  PASS: bare net pours the last copper layer; NET:LAYER explicit")


def test_islands_outrank_hpwl():
    whole = _cand(1, hpwl=120.0, plane_islands=1, plane_neck=50.0)
    split = _cand(2, hpwl=100.0, plane_islands=2, plane_neck=50.0)
    assert rank_key(whole) < rank_key(split), (
        "a split pour must rank below a whole one even at better hpwl -- "
        "a measured plane loss outranks a proxy")
    print("  PASS: plane_islands ranks before hpwl")


def test_neck_breaks_hpwl_ties_only():
    a = _cand(1, hpwl=100.0, plane_islands=1, plane_neck=40.0)
    b = _cand(2, hpwl=100.0, plane_islands=1, plane_neck=90.0)
    assert rank_key(a) < rank_key(b), "equal hpwl: fewer necks wins"
    c = _cand(1, hpwl=90.0, plane_islands=1, plane_neck=500.0)
    d = _cand(2, hpwl=100.0, plane_islands=1, plane_neck=1.0)
    assert rank_key(c) < rank_key(d), "hpwl still outranks the neck sum"
    print("  PASS: plane_neck is a below-hpwl tiebreak")


def test_absent_metrics_leave_ranking_unchanged():
    a = _cand(1, hpwl=100.0)
    b = _cand(2, hpwl=110.0)
    assert rank_key(a) < rank_key(b)
    # By NAME, not by index: a positional check keeps passing after a reorder
    # even when it is no longer reading the term it means.
    assert rank_key(a).plane_islands == 0, \
        "no --plane-score -> islands term is 0"
    print("  PASS: flag off is inert")


def test_portfolio_flag_exists():
    import place_portfolio
    p = place_portfolio.build_parser()
    a = p.parse_args([BOARD, '--out-dir', 'x', '--plane-score', 'GND'])
    assert a.plane_score == ['GND']
    # The budget is GONE (#713 item 1). Pinned in tests/test_713_removed_flags
    # the way #621 pinned --deadline: a removed flag exits argparse rc 2, which
    # a harness reads as an engine failure, so the removal is asserted rather
    # than assumed.
    assert not hasattr(a, 'plane_score_budget'), \
        "a wall clock must not decide which candidate wins"
    print("  PASS: --plane-score parses; --plane-score-budget is gone")


def test_score_smoke_on_splitflap():
    from kicad_exact_fill import find_kicad_python
    if find_kicad_python() is None:
        print("  SKIP: no KiCad python for the refill")
        return
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    from plane_score import plane_fragility_score
    sc = plane_fragility_score(BOARD, [('GND', 'B.Cu')])
    assert sc is not None, "refill available but score came back None"
    assert sc['islands'] >= 1 and sc['neck_sum'] > 0, sc
    assert 'GND:B.Cu' in sc['per_net']
    print(f"  PASS: splitflap GND pour scored "
          f"(islands={sc['islands']}, neck={sc['neck_sum']})")


if __name__ == '__main__':
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        print(f"--- {fn.__name__}")
        fn()
    print("ALL PASS")
