"""
Ratsnest metrics are exported, and screen candidates before routing (issue #504).

The quench cost function already computed airwire length and crossings on every
pass, and #457 added HPWL — then printed all of it and threw it away. Nothing
outside the optimizer could see what a placement run achieved, so nothing could
gate on it.

`quench(metrics_out=...)` now hands those numbers back, `place_optimize.py` emits
them as a JSON_SUMMARY, and `place_route_loop.py` records them per round and can
use them to skip a routing run that is very unlikely to help.

Reading the numbers: `crossings` and `hpwl` are UNWEIGHTED (crossings is a raw
count by contract; hpwl is pure pad geometry), so they compare across calls.
`length` and `total` are scaled by net_weights, so they only compare between the
before/after of one call — which is why the screen thresholds on the first two.
"""

import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split (was tests/py_placer, which does not exist)

# #522 reorg + skill merge: engine -> py_router/, placer -> py_placer/,
# board_score.py -> the placement-and-routing skill. Without these roots the
# imports below raise and the test never runs (it reports as a failure while
# asserting nothing).
_R522 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p522 in ('py_router', 'py_placer',
              os.path.join('.claude', 'skills',
                           'plan-pcb-placement-and-routing', 'scripts')):
    _d522 = os.path.join(_R522, _p522)
    if _d522 not in sys.path:
        sys.path.insert(0, _d522)


from kicad_parser import parse_kicad_pcb
from place_route_loop import _ratsnest_screen, better
from placement.quench import QuenchState, quench

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, 'kicad_files', 'interf_u_unrouted.kicad_pcb')
KN = dict(clearance=0.25, board_edge_clearance=0.55, crossing_penalty=10.0,
          halo_base=0.3, halo_coef=0.15, halo_weight=1.0,
          edge_halo=1.0, edge_weight=1.0, grid_step=0.05)


# --- the out-param ------------------------------------------------------------

def test_metrics_out_is_populated_and_matches_total_cost():
    """The exported numbers must be the ones the optimizer actually acted on,
    not a second implementation that can drift."""
    m = {}
    quench(parse_kicad_pcb(BOARD), pcb_file=BOARD, max_displacement=2.0,
           step=1.0, max_passes=1, metrics_out=m)
    assert set(m) == {'before', 'after', 'legality'}, f"keys: {sorted(m)}"
    for phase in ('before', 'after'):
        for key in ('total', 'length', 'crossings', 'halo', 'edge', 'hpwl'):
            assert key in m[phase], f"{phase} missing {key}"
    # 'before' is the untouched board, so an independent QuenchState must agree.
    st = QuenchState(parse_kicad_pcb(BOARD), BOARD, **KN)
    ref = st.total_cost()
    assert m['before']['crossings'] == ref['crossings'], \
        f"before.crossings {m['before']['crossings']} != {ref['crossings']}"
    assert abs(m['before']['hpwl'] - ref['hpwl']) < 1e-9
    for key in ('overlap_area', 'oob_count', 'oob_amount', 'oob_area'):
        assert key in m['legality'], f"legality missing {key}"
    print(f"  PASS: crossings {m['before']['crossings']} -> {m['after']['crossings']}, "
          f"hpwl {m['before']['hpwl']:.1f} -> {m['after']['hpwl']:.1f}")


def test_metrics_out_is_optional_and_non_breaking():
    """Every existing caller passes no metrics_out and must still get a list."""
    out = quench(parse_kicad_pcb(BOARD), pcb_file=BOARD, max_displacement=2.0,
                 step=1.0, max_passes=1)
    assert isinstance(out, list), f"return type changed: {type(out).__name__}"
    assert all(set(p) == {'reference', 'new_x', 'new_y', 'new_rotation'}
               for p in out), "placement dict shape changed"


def test_quench_only_improves_its_own_objective():
    """Sanity for anything screening on these: the optimizer is monotone in
    `total`, but NOT necessarily in any single term — which is exactly why a
    per-term screen is meaningful."""
    m = {}
    quench(parse_kicad_pcb(BOARD), pcb_file=BOARD, max_displacement=2.0,
           step=1.0, max_passes=2, metrics_out=m)
    assert m['after']['total'] <= m['before']['total'] + 1e-6, \
        f"total regressed: {m['before']['total']} -> {m['after']['total']}"


# --- the screen ---------------------------------------------------------------

def test_screen_disabled_by_default():
    before = {'crossings': 100, 'hpwl': 1000.0}
    after = {'crossings': 500, 'hpwl': 9000.0}      # catastrophically worse
    for pct in (0, 0.0, None):
        skip, _ = _ratsnest_screen(before, after, pct)
        assert not skip, f"screen fired with pct={pct!r}; it must be opt-in"


def test_screen_fires_on_a_regression_beyond_the_threshold():
    before = {'crossings': 100, 'hpwl': 1000.0}
    after = {'crossings': 110, 'hpwl': 1000.0}      # +10% crossings
    skip, why = _ratsnest_screen(before, after, 5.0)
    assert skip, f"expected a skip at +10% vs 5% threshold: {why}"
    assert 'crossings' in why and '+10.0%' in why, f"note not auditable: {why}"
    skip, why = _ratsnest_screen(before, after, 20.0)
    assert not skip, f"+10% must pass a 20% threshold: {why}"


def test_screen_fires_on_hpwl_too():
    before = {'crossings': 100, 'hpwl': 1000.0}
    after = {'crossings': 100, 'hpwl': 1200.0}      # +20% hpwl
    skip, why = _ratsnest_screen(before, after, 5.0)
    assert skip and 'hpwl' in why, f"hpwl regression not screened: {why}"


def test_screen_lets_an_improvement_through():
    before = {'crossings': 100, 'hpwl': 1000.0}
    after = {'crossings': 80, 'hpwl': 950.0}
    skip, why = _ratsnest_screen(before, after, 5.0)
    assert not skip, f"an improving candidate was screened: {why}"
    assert '-20.0%' in why, f"note should still report the numbers: {why}"


def test_screen_always_reports_its_numbers():
    """The issue asks that the decision be auditable every round."""
    _, why = _ratsnest_screen({'crossings': 10, 'hpwl': 5.0},
                              {'crossings': 10, 'hpwl': 5.0}, 5.0)
    assert 'crossings 10->10' in why and 'hpwl 5->5' in why, why


def test_screen_is_inert_without_metrics():
    """A quench that filled nothing must never cause a skip."""
    for b, a in (({}, {}), ({'crossings': 1}, {}), (None, None)):
        skip, _ = _ratsnest_screen(b, a, 5.0)
        assert not skip


def test_screen_handles_a_zero_baseline():
    """crossings 0 -> 0 is not a regression; 0 -> n is."""
    skip, _ = _ratsnest_screen({'crossings': 0, 'hpwl': 1.0},
                               {'crossings': 0, 'hpwl': 1.0}, 5.0)
    assert not skip, "0 -> 0 must not be a regression"
    skip, _ = _ratsnest_screen({'crossings': 0, 'hpwl': 1.0},
                               {'crossings': 3, 'hpwl': 1.0}, 5.0)
    assert skip, "0 -> 3 must be a regression"


# --- the reporting plumbing ---------------------------------------------------

def test_better_ignores_the_ratsnest_keys():
    """Report-only, exactly like pad_pairs_*: reworking the comparator is #458."""
    a = {'failures': 1, 'iterations': 100, 'ratsnest_crossings': 999,
         'ratsnest_hpwl': 9e9}
    b = {'failures': 2, 'iterations': 100, 'ratsnest_crossings': 1,
         'ratsnest_hpwl': 1.0}
    assert better(a, b), "failures must still decide, regardless of ratsnest"
    c = dict(a, failures=2)
    assert not better(c, b), "equal failures + equal iterations must not flip"


def test_place_optimize_emits_a_parseable_json_summary():
    import tempfile
    fd, out = tempfile.mkstemp(suffix='.kicad_pcb')
    os.close(fd)
    r = subprocess.run([sys.executable, os.path.join(ROOT, 'py_placer', 'place_optimize.py'),
                        BOARD, out, '--max-displacement', '2', '--step', '1',
                        '--max-passes', '1'],
                       capture_output=True, text=True, cwd=ROOT)
    assert r.returncode == 0, f"place_optimize failed:\n{r.stdout[-800:]}{r.stderr[-400:]}"
    lines = [l for l in r.stdout.splitlines() if l.startswith('JSON_SUMMARY: ')]
    assert lines, f"no JSON_SUMMARY emitted:\n{r.stdout[-800:]}"
    d = json.loads(lines[-1].split('JSON_SUMMARY: ', 1)[1])
    for key in ('parts_moved', 'crossings_before', 'crossings_after',
                'hpwl_before', 'hpwl_after', 'airwire_length_before',
                'airwire_length_after', 'overlap_area', 'oob_count'):
        assert key in d, f"summary missing {key}: {sorted(d)}"
    assert isinstance(d['crossings_after'], int)
    print(f"  PASS: summary crossings {d['crossings_before']} -> {d['crossings_after']}")
    for p in (out, os.path.splitext(out)[0] + '.kicad_pro'):
        if os.path.exists(p):
            os.unlink(p)


TESTS = [
    test_metrics_out_is_populated_and_matches_total_cost,
    test_metrics_out_is_optional_and_non_breaking,
    test_quench_only_improves_its_own_objective,
    test_screen_disabled_by_default,
    test_screen_fires_on_a_regression_beyond_the_threshold,
    test_screen_fires_on_hpwl_too,
    test_screen_lets_an_improvement_through,
    test_screen_always_reports_its_numbers,
    test_screen_is_inert_without_metrics,
    test_screen_handles_a_zero_baseline,
    test_better_ignores_the_ratsnest_keys,
    test_place_optimize_emits_a_parseable_json_summary,
]


if __name__ == '__main__':
    for t in TESTS:
        print(f"--- {t.__name__}")
        t()
    print("ALL PASS")
