#!/usr/bin/env python3
"""The portfolio applies rule 1 of the acceptance conjunction, K-way.

Run-7 S6: SKILL.md claimed the portfolio "pre-applies the first two
[acceptance-rule parts] as its hard gates". As measured, the hard gates were
legality + intent only -- the METRIC clause (crossings/hpwl no worse than the
baseline) was never checked, and a candidate worse on both proxies could top
the slate and ship through JSON_SUMMARY['best'] unremarked.

Contract under test:
  * rule1_check is pure and per-candidate-vs-baseline;
  * select_best never silently picks a violator (probe rank outranks static;
    the baseline always passes against itself; all-violators -> None);
  * violators are ANNOTATED (gates.rule1 + doc.rule1_violators), not gated --
    they stay ranked and probed;
  * --full-probe exists, and probe rows are labeled with probe_kind.
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split

from placement.portfolio import Candidate, rule1_check, select_best  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')


def _cand(index, crossings, hpwl, board='b.kicad_pcb'):
    c = Candidate(index=index, strategy='jitter', board=board)
    c.metrics = {'crossings': crossings, 'hpwl': hpwl}
    return c


def test_rule1_check_measures_hpwl_against_the_baseline_row():
    """The hpwl clause, unchanged. The crossings clause was WITHDRAWN by #789.

    This function used to assert that a candidate worse on crossings alone was
    barred, and that one worse on both listed two violations. Both assertions
    are now inverted, deliberately: `docs/placement-predictors.md` rule 2
    pre-registered the exit criterion, a six-board slate run satisfied it on
    two boards, and the clause is gone. The direction survives in
    `rule1_advisory` -- see `test_rule1_advisory_keeps_the_withdrawn_direction`
    below, and `tests/test_789_rule1_withdrawal.py` for the recorded evidence.
    """
    base = _cand(0, 50, 600.0)
    assert rule1_check(_cand(1, 45, 590.0), base) == []
    assert rule1_check(_cand(2, 50, 600.0), base) == [], "equal is no worse"
    assert rule1_check(_cand(3, 60, 590.0), base) == [], \
        "#789: worse on crossings ALONE no longer bars"
    v = rule1_check(_cand(4, 45, 700.0), base)
    assert len(v) == 1 and 'hpwl' in v[0], v
    v = rule1_check(_cand(5, 60, 700.0), base)
    assert len(v) == 1 and 'hpwl' in v[0], \
        f"worse on both is barred by the hpwl clause alone now: {v}"
    print("  PASS: rule1_check prices hpwl against the baseline; crossings is "
          "withdrawn (#789)")


def test_rule1_advisory_keeps_the_withdrawn_direction():
    """Rule 2 said the withdrawal keeps its MEASURED DIRECTION.

    A clause that stops measuring is a deleted clause: a later reader could not
    see what the bar used to say, nor notice the corpus moving under it.
    """
    from placement.portfolio import rule1_advisory
    base = _cand(0, 50, 600.0)
    adv = rule1_advisory(_cand(3, 60, 590.0), base)
    assert len(adv) == 1 and 'crossings' in adv[0], adv
    assert 'WITHDRAWN' in adv[0], \
        f"the advisory must say it does not bar, or it reads as one: {adv}"
    assert rule1_advisory(_cand(1, 45, 590.0), base) == [], \
        "a candidate with FEWER crossings has nothing to report"
    assert rule1_advisory(_cand(2, 50, 600.0), base) == [], "equal is no worse"
    # It is ADVISORY, so it must never reach the violator set select_best reads.
    assert rule1_check(_cand(3, 60, 590.0), base) == []
    print("  PASS: rule1_advisory reports the withdrawn direction and bars "
          "nothing")


def test_select_best_never_silently_picks_a_violator():
    # Window probe ranked a violator first: best skips to the next passer.
    assert select_best([3, 1, 2], [1, 2, 3], {3}) == 1
    # Probe ranking outranks static.
    assert select_best([2, 1], [1, 2], set()) == 2
    # The baseline (0) always passes -- "keep what you have" is first-class.
    assert select_best([0, 3], [3], {3}) == 0
    # Everything ranked violates -> None (caller falls back to baseline).
    assert select_best([3, 4], [4, 3], {3, 4}) is None
    # No probe tier: first static passer.
    assert select_best([], [5, 6], {5}) == 6
    print("  PASS: select_best skips violators, probe outranks static")


def test_full_probe_flag_and_labels_exist():
    import place_portfolio
    p = place_portfolio.build_parser()
    a = p.parse_args([BOARD, '--out-dir', 'x', '--full-probe'])
    assert a.full_probe is True
    src = open(os.path.join(ROOT, 'py_placer', 'place_portfolio.py'), encoding='utf-8').read()
    assert "probe_kind" in src and "'full'" in src, (
        "probe rows must be labeled window vs full")
    assert "ranking_full" in src
    print("  PASS: --full-probe flag + probe_kind labels present")


def test_end_to_end_annotation_reaches_portfolio_json():
    if not os.path.isfile(BOARD):
        print("  SKIP: fixture missing")
        return
    with tempfile.TemporaryDirectory() as d:
        out = os.path.join(d, 'pf')
        r = subprocess.run(
            [sys.executable, '-X', 'utf8',
             os.path.join(ROOT, 'py_placer', 'place_portfolio.py'), BOARD,
             '--out-dir', out, '--seed', '0', '--candidates', '3',
             '--keep', '1', '--route-top', '0', '--no-render'],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', cwd=ROOT,
            env=dict(os.environ, PYTHONIOENCODING='utf-8'))
        assert r.returncode in (0, 4), (r.stdout + r.stderr)[-500:]
        doc = json.load(open(os.path.join(out, 'portfolio.json'),
                             encoding='utf-8'))
        assert 'rule1_violators' in doc, "doc must annotate rule-1 violators"
        assert 'ranking_full' in doc
        for c in doc['candidates']:
            if c['board']:
                assert 'rule1' in c['gates'], (
                    f"cand {c['index']} missing gates.rule1")
        line = next(l for l in r.stdout.splitlines()
                    if l.startswith('JSON_SUMMARY:'))
        s = json.loads(line.split(':', 1)[1])
        assert 'rule1_excluded' in s and 'probe_kind' in s
        best = s['best']
        if best is not None and best != 0:
            assert best not in s['rule1_excluded'], (
                f"best={best} is a rule-1 violator -- the S6 bug is back")
    print("  PASS: rule-1 annotation + summary keys survive a real run")


if __name__ == '__main__':
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        print(f"--- {fn.__name__}")
        fn()
    print("ALL PASS")
