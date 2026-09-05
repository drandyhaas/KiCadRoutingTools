"""
Tests for how place_route_loop steers the quench (issue #458).

The loop widens --max-displacement 1.5x after a rejected round. Since #455
gave quench a swap cap that DEFAULTS to max_displacement, that widening used
to widen swap teleports in lockstep: a 3mm base reaches 15mm after four
rejections, which is the #430 stranding failure coming back through the loop.
The loop now passes swap_max_displacement explicitly and holds it at the base
value while the nudge radius grows, and it plumbs --no-rotate / --no-swap /
--verbose through to quench.

Nothing is routed here. The routing step, the quench and the writer are
replaced with recorders, so the whole file is a unit test: no router binary,
no board is written, and it stays in the run_all.py --fast lane.
"""

import inspect
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_tools'))  # #522

import place_route_loop as prl
from placement.quench import quench as real_quench


def _loop_board():
    """One movable 2-pad part C1 on net NA plus a locked anchor J1 on the
    same net, inside a (100,80)-(200,120) outline. The loop only needs the
    board to parse and to yield one movable target ref for net NA."""
    body = '(kicad_pcb\n\t(version 20241229)\n'
    body += '\t(net 0 "")\n\t(net 1 "NA")\n'
    body += '''\t(gr_rect
\t\t(start 100 80)
\t\t(end 200 120)
\t\t(stroke
\t\t\t(width 0.1)
\t\t\t(type solid)
\t\t)
\t\t(layer "Edge.Cuts")
\t\t(uuid "edge1")
\t)
\t(footprint "test:CAP2P"
\t\t(layer "F.Cu")
\t\t(uuid "fp-C1")
\t\t(at 150 100)
\t\t(property "Reference" "C1"
\t\t\t(at 0 0)
\t\t)
\t\t(pad "1" smd rect
\t\t\t(at -0.5 0)
\t\t\t(size 0.8 0.8)
\t\t\t(layers "F.Cu")
\t\t\t(net 1 "NA")
\t\t\t(uuid "p1-C1")
\t\t)
\t)
\t(footprint "test:PIN"
\t\t(layer "F.Cu")
\t\t(locked yes)
\t\t(uuid "fp-J1")
\t\t(at 170 100)
\t\t(property "Reference" "J1"
\t\t\t(at 0 0)
\t\t)
\t\t(pad "1" smd rect
\t\t\t(at 0 0)
\t\t\t(size 1 1)
\t\t\t(layers "F.Cu")
\t\t\t(net 1 "NA")
\t\t\t(uuid "p1-J1")
\t\t)
\t)
)
'''
    fd, path = tempfile.mkstemp(suffix='.kicad_pcb')
    with os.fdopen(fd, 'w') as f:
        f.write(body)
    return path, tempfile.mkdtemp()


def _run_loop(extra_args, rounds=4, quench_result=None,
              quench_metrics=None, route_calls=None):
    """Run main() with the routing step, the quench and the writer replaced.
    Returns the list of kwargs quench was called with, one entry per round.

    quench_metrics: optional {'before':..., 'after':...} the fake quench writes
    into the caller's metrics_out, so the #504 ratsnest screen has numbers.
    route_calls: optional list that receives one entry per run_route call, so a
    test can assert the screen actually SKIPPED the routing run."""
    calls = []
    placements = ([{'reference': 'C1', 'new_x': 151.0, 'new_y': 100.0,
                    'new_rotation': 0.0}] if quench_result is None
                  else quench_result)

    def fake_quench(pcb_data, **kw):
        calls.append(dict(kw))
        if quench_metrics is not None and kw.get('metrics_out') is not None:
            kw['metrics_out'].update(quench_metrics)
        return placements

    # **kw, not a fixed signature: this fake stands in for run_route, and
    # pinning its parameter list here makes an unrelated addition to the real
    # one (json_file, for --accept-cmd's judge) fail as a steering regression.
    # What this test is about is WHICH BOARD each round routes, nothing else.
    def fake_run_route(pcb_file, routed_file, route_args, log_file, **kw):
        if route_calls is not None:
            route_calls.append(pcb_file)
        # Every candidate scores exactly like round 0, so better() is False
        # and every round is REJECTED: the cap widens on every round.
        return {'failures': 2, 'failed_nets': ['NA'], 'blockers': [],
                'iterations': 1000, 'vias': 0}

    board, work = _loop_board()
    saved = (prl.quench, prl.run_route, prl.write_placed_output, sys.argv)
    prl.quench = fake_quench
    prl.run_route = fake_run_route
    prl.write_placed_output = lambda src, dst, pl: shutil.copy(src, dst)
    sys.argv = ['place_route_loop.py', board,
                os.path.join(work, 'out.kicad_pcb'),
                '--route-args', '--nets "*"', '--rounds', str(rounds),
                '--max-displacement', '3.0', '--work-dir', work] + extra_args
    try:
        prl.main()
    finally:
        (prl.quench, prl.run_route, prl.write_placed_output,
         sys.argv) = saved
        os.unlink(board)
        shutil.rmtree(work, ignore_errors=True)
    return calls


def test_swap_cap_held_while_displacement_widens():
    """The headline invariant: four rejected rounds widen the nudge cap from
    3mm to 10.1mm while the swap cap stays at the base 3mm."""
    calls = _run_loop([])
    assert len(calls) == 4, f"expected one quench per round, got {len(calls)}"
    assert [c['max_displacement'] for c in calls] == [3.0, 4.5, 6.75, 10.125]
    assert [c['swap_max_displacement'] for c in calls] == [3.0, 3.0, 3.0, 3.0]
    for c in calls:
        # quench() raises ValueError if this is ever violated.
        assert c['swap_max_displacement'] <= c['max_displacement'] + 1e-9


def test_swap_cap_held_when_quench_finds_nothing():
    """The other widening site: an empty quench result widens the cap without
    a candidate route, and must not widen swaps either."""
    calls = _run_loop([], quench_result=[])
    assert [c['max_displacement'] for c in calls] == [3.0, 4.5, 6.75, 10.125]
    assert [c['swap_max_displacement'] for c in calls] == [3.0] * 4


def test_swap_cap_flag_overrides_the_base():
    calls = _run_loop(['--swap-max-displacement', '1.0'])
    assert [c['swap_max_displacement'] for c in calls] == [1.0] * 4


def test_rotate_and_swap_flags_reach_quench():
    """--no-rotate / --no-swap / --verbose exist on the loop and arrive at
    quench; without them both move types stay enabled."""
    calls = _run_loop([])
    assert all(c['allow_rotations'] is True and c['allow_swaps'] is True
               and c['verbose'] is False for c in calls), \
        "defaults must keep both move types enabled and stay quiet"
    calls = _run_loop(['--no-rotate', '--no-swap', '--verbose'])
    assert all(c['allow_rotations'] is False and c['allow_swaps'] is False
               and c['verbose'] is True for c in calls)


def test_kwargs_match_the_real_quench_signature():
    """Guards the recorder itself: binding the captured kwargs against the
    real signature fails on a renamed keyword that a stub would swallow."""
    sig = inspect.signature(real_quench)
    for c in _run_loop([]):
        sig.bind(None, **c)


def test_swap_cap_above_displacement_is_rejected():
    """Mirrors place_optimize.py's argparse check rather than letting the
    value reach quench and raise ValueError from inside a round."""
    try:
        _run_loop(['--swap-max-displacement', '9.0'])
    except SystemExit as exc:
        assert exc.code != 0
    else:
        raise AssertionError("--swap-max-displacement 9.0 must be rejected "
                             "against a 3.0mm --max-displacement")


# --- the round tally (issue #458 item 1) -------------------------------
#
# route.py runs an end-of-run reconciliation pass exactly when the first pass
# left failures, and prints a second JSON_SUMMARY scoped to the retried nets.
# The loop used to read the FIRST summary, so every net the reconciliation
# recovered still counted as failed. The logs below are trimmed to the lines
# the loop actually parses.

LOG_TWO_PASS = '''Frontier blocking analysis:
  1. /VBUS: 46 (31.7%), 12 uniq (26%)
  2. /GND: 20 (13.8%), 4 uniq (20%)
JSON_SUMMARY: {"failed_single": ["/A", "/B"], "failed_multipoint": \
[{"net_name": "/C", "failed_pads": [{"component_ref": "U1", \
"pad_number": "3"}, {"component_ref": "U2", "pad_number": "7"}]}], \
"multipoint_pads_total": 20, "multipoint_pads_connected": 18, \
"total_iterations": 1000, "total_vias": 300, "total_time": 12.5}

Final reconciliation: retrying 3 incomplete net(s) against the finished \
board: /A, /B, /C
Frontier blocking analysis:
  1. /MD1: 7 (9.1%), 2 uniq (28%)
JSON_SUMMARY: {"failed_single": ["/B"], "failed_multipoint": \
[{"net_name": "/C", "failed_pads": [{"component_ref": "U1", \
"pad_number": "3"}]}], "multipoint_pads_total": 12, \
"multipoint_pads_connected": 11, "total_iterations": 40, "total_vias": 7, \
"total_time": 0.4}
Note: the JSON_SUMMARY above covers only the reconciliation subset; the \
run's full tally is the earlier JSON_SUMMARY plus these recoveries.
'''

LOG_ONE_PASS = '''  Single-ended:  4/4 routed
JSON_SUMMARY: {"failed_single": [], "failed_multipoint": [], \
"multipoint_pads_total": 20, "multipoint_pads_connected": 20, \
"total_iterations": 777, "total_vias": 42, "total_time": 9.0}
'''

# The reconciliation pass may escalate to rip authority over hinted blockers,
# and its own coverage gate then reports a net that was never in the retry
# set: a net the reconciliation itself broke.
LOG_RECON_BROKE = '''JSON_SUMMARY: {"failed_single": ["/A"], \
"failed_multipoint": [], "multipoint_pads_total": 20, \
"multipoint_pads_connected": 20, "total_iterations": 900, "total_vias": 50, \
"total_time": 8.0}

Final reconciliation: retrying 1 incomplete net(s) against the finished \
board: /A
JSON_SUMMARY: {"failed_single": [], "failed_multipoint": \
[{"net_name": "/GND_RET", "failed_pads": [{"component_ref": "J1", \
"pad_number": "1"}, {"component_ref": "J1", "pad_number": "2"}]}], \
"coverage_gate_nets": ["/GND_RET"], "multipoint_pads_total": 6, \
"multipoint_pads_connected": 6, "total_iterations": 60, "total_vias": 3, \
"total_time": 0.6}
'''

# The sub-run prints its summary BEFORE the board is written, so an exception
# during the write leaves the pass-1 board on disk under a summary that
# advertises recoveries. route.py:2295 wraps that message in RED/RESET.
LOG_RECON_ABORTED = '''JSON_SUMMARY: {"failed_single": ["/A", "/B"], \
"failed_multipoint": [], "multipoint_pads_total": 10, \
"multipoint_pads_connected": 10, "total_iterations": 500, "total_vias": 20, \
"total_time": 5.0}

Final reconciliation: retrying 2 incomplete net(s) against the finished \
board: /A, /B
JSON_SUMMARY: {"failed_single": [], "failed_multipoint": [], \
"multipoint_pads_total": 4, "multipoint_pads_connected": 4, \
"total_iterations": 30, "total_vias": 2, "total_time": 0.3}
\033[91m  final reconciliation pass failed: [Errno 28] No space left on \
device\033[0m
'''

LOG_RECON_NO_SUMMARY = '''JSON_SUMMARY: {"failed_single": ["/A"], \
"failed_multipoint": [], "multipoint_pads_total": 0, \
"multipoint_pads_connected": 0, "total_iterations": 300, "total_vias": 11, \
"total_time": 3.0}

Final reconciliation: retrying 1 incomplete net(s) against the finished \
board: /A
All nets are already fully connected - nothing to route!
'''


def _metrics_for(log_text, rc=0):
    """Drive run_route against a canned route.py log, no child process."""
    work = tempfile.mkdtemp()
    calls = []

    def fake(cmd, log_file):
        calls.append(list(cmd))
        with open(log_file, 'w', encoding='utf-8') as f:
            f.write(log_text)
        return rc

    saved = prl._run_route_cmd
    prl._run_route_cmd = fake
    try:
        metrics = prl.run_route(os.path.join(work, 'in.kicad_pcb'),
                                os.path.join(work, 'out.kicad_pcb'),
                                '--nets *', os.path.join(work, 'route.log'))
    finally:
        prl._run_route_cmd = saved
        shutil.rmtree(work, ignore_errors=True)
    return metrics, calls


# --- the #432 keys across the merge (pad_pairs_* and blockers) ---------
#
# The pad-pair tallies are whole-board in the first summary but
# reconcile-subset-scoped in the sub-run's, so last-wins would swap a 40/30
# whole-board reading for the sub-run's subset numbers (or 0/0). Rebuilt:
# total from pass 1, connected = total - the deficit still open at END.

LOG_PP_TWO_PASS = '''JSON_SUMMARY: {"failed_single": ["/A"], \
"failed_multipoint": [{"net_name": "/B", "failed_pads": []}, \
{"net_name": "/C", "failed_pads": []}], "multipoint_pads_total": 20, \
"multipoint_pads_connected": 14, "total_iterations": 1000, "total_vias": 300, \
"total_time": 12.5, "pad_pairs_connected": 30, "pad_pairs_total": 40, \
"pad_pairs_open": [{"net": "/A", "pairs_connected": 0, "pairs_total": 2, \
"outcome": "open", "open_subtype": "unrouted"}, {"net": "/B", \
"pairs_connected": 2, "pairs_total": 5, "outcome": "open", \
"open_subtype": "partial"}, {"net": "/C", "pairs_connected": 5, \
"pairs_total": 8, "outcome": "open", "open_subtype": "partial"}]}

Final reconciliation: retrying 3 incomplete net(s) against the finished \
board: /A, /B, /C
JSON_SUMMARY: {"failed_single": [], "failed_multipoint": \
[{"net_name": "/B", "failed_pads": []}, {"net_name": "/C", \
"failed_pads": []}], "multipoint_pads_total": 15, \
"multipoint_pads_connected": 10, "total_iterations": 40, "total_vias": 7, \
"total_time": 0.4, "pad_pairs_connected": 9, "pad_pairs_total": 15, \
"pad_pairs_open": [{"net": "/B", "pairs_connected": 3, "pairs_total": 5, \
"outcome": "open", "open_subtype": "partial"}, {"net": "/C", \
"pairs_connected": 5, "pairs_total": 8, "outcome": "open", \
"open_subtype": "partial"}]}
'''

# Sub-run recovered /A and emitted its own (empty-ish) blockers key: pinned
# that an explicit empty list does NOT regress to the whole-log regex, which
# would scrape the transient pass-1 analysis of a net that later routed.
LOG_BLOCKERS_RECOVERED_ALL = '''Frontier blocking analysis:
  1. /VBUS: 46 (31.7%), 12 uniq (26%)
JSON_SUMMARY: {"failed_single": ["/A"], "failed_multipoint": [], \
"multipoint_pads_total": 4, "multipoint_pads_connected": 4, \
"total_iterations": 500, "total_vias": 20, "total_time": 5.0, \
"blockers": [{"net": "/A", "stage": "single_ended", "blocked_by": \
[{"net": "/VBUS", "blocked_count": 46}]}]}

Final reconciliation: retrying 1 incomplete net(s) against the finished \
board: /A
JSON_SUMMARY: {"failed_single": [], "failed_multipoint": [], \
"multipoint_pads_total": 2, "multipoint_pads_connected": 2, \
"total_iterations": 30, "total_vias": 2, "total_time": 0.3}
'''

# Sub-run kept /B failed but emitted no blockers key of its own: pass 1's
# attribution survives for /B only; recovered /A's entry dies with it.
LOG_BLOCKERS_PARTIAL = '''JSON_SUMMARY: {"failed_single": ["/A", "/B"], \
"failed_multipoint": [], "multipoint_pads_total": 4, \
"multipoint_pads_connected": 4, "total_iterations": 500, "total_vias": 20, \
"total_time": 5.0, "blockers": [{"net": "/A", "stage": "single_ended", \
"blocked_by": [{"net": "/VBUS", "blocked_count": 46}]}, {"net": "/B", \
"stage": "phase3", "blocked_by": [{"net": "/MD1", "blocked_count": 7}]}]}

Final reconciliation: retrying 2 incomplete net(s) against the finished \
board: /A, /B
JSON_SUMMARY: {"failed_single": ["/B"], "failed_multipoint": [], \
"multipoint_pads_total": 2, "multipoint_pads_connected": 2, \
"total_iterations": 30, "total_vias": 2, "total_time": 0.3}
'''

# Sub-run's summary carries no pad-pair keys at all (its emission is
# defensively try/except'd): pass 1's numbers are the newest that exist.
LOG_PP_SUBRUN_WITHOUT_KEYS = '''JSON_SUMMARY: {"failed_single": ["/A"], \
"failed_multipoint": [], "multipoint_pads_total": 4, \
"multipoint_pads_connected": 4, "total_iterations": 500, "total_vias": 20, \
"total_time": 5.0, "pad_pairs_connected": 30, "pad_pairs_total": 40, \
"pad_pairs_open": [{"net": "/A", "pairs_connected": 0, "pairs_total": 2, \
"outcome": "open", "open_subtype": "unrouted"}]}

Final reconciliation: retrying 1 incomplete net(s) against the finished \
board: /A
JSON_SUMMARY: {"failed_single": ["/A"], "failed_multipoint": [], \
"multipoint_pads_total": 2, "multipoint_pads_connected": 2, \
"total_iterations": 30, "total_vias": 2, "total_time": 0.3}
'''


def test_pad_pairs_keep_the_whole_board_denominator():
    """Pass 1 graded the whole board 30/40 (open: /A 0/2, /B 2/5, /C 5/8);
    the sub-run recovered /A and half of /B's deficit. Last-wins would
    report the subset's 9/15; the honest whole-board reading is
    40 - (2 + 3) = 35 connected of 40."""
    metrics, _ = _metrics_for(LOG_PP_TWO_PASS)
    assert metrics['pad_pairs_total'] == 40, metrics
    assert metrics['pad_pairs_connected'] == 35, metrics


def test_pad_pairs_survive_a_sub_run_without_the_keys():
    """A sub-run summary with no pad-pair keys must not zero the metrics."""
    metrics, _ = _metrics_for(LOG_PP_SUBRUN_WITHOUT_KEYS)
    assert metrics['pad_pairs_total'] == 40, metrics
    assert metrics['pad_pairs_connected'] == 30, metrics


def test_empty_structured_blockers_do_not_regress_to_the_regex():
    """The reconciliation recovered every failure; the merged blockers are
    an explicit empty set, NOT the whole-log regex scrape (which would
    resurrect /VBUS, the blocker of a net that ultimately routed)."""
    metrics, _ = _metrics_for(LOG_BLOCKERS_RECOVERED_ALL)
    assert metrics['failures'] == 0, metrics
    assert metrics['blockers'] == [], metrics


def test_blockers_filtered_to_still_failed_when_sub_run_omits_the_key():
    """/B is still failed and keeps pass 1's attribution (/MD1); recovered
    /A's blocker (/VBUS) dies with it."""
    metrics, _ = _metrics_for(LOG_BLOCKERS_PARTIAL)
    assert set(metrics['failed_nets']) == {'/B'}, metrics['failed_nets']
    assert metrics['blockers'] == ['/MD1'], metrics


def test_reconciliation_recoveries_are_not_counted_as_failures():
    """/A was recovered by the reconciliation pass, /B is still open and /C
    recovered one of its two open pads. Reading the first summary gave 4."""
    metrics, _ = _metrics_for(LOG_TWO_PASS)
    assert metrics['failures'] == 2, metrics
    assert '/A' not in metrics['failed_nets'], metrics['failed_nets']
    assert set(metrics['failed_nets']) == {'/B', '/C'}, metrics['failed_nets']


def test_effort_counters_are_summed_across_passes():
    """Both passes are work this placement cost the router, so better()'s
    iteration tiebreak has to see both."""
    metrics, _ = _metrics_for(LOG_TWO_PASS)
    assert metrics['iterations'] == 1040, metrics
    assert metrics['vias'] == 307, metrics


def test_blockers_are_unioned_across_both_passes():
    """The blocker scrape reads the whole log, so it already spans both
    passes. Pinned so a future switch to a summary key cannot narrow it."""
    metrics, _ = _metrics_for(LOG_TWO_PASS)
    assert metrics['blockers'] == ['/GND', '/MD1', '/VBUS'], metrics


def test_single_summary_run_is_unchanged():
    """A clean run has no reconciliation pass and one summary, which is both
    the first and the last: same numbers as before the fix, effort not
    doubled."""
    metrics, _ = _metrics_for(LOG_ONE_PASS)
    assert metrics['failures'] == 0
    assert metrics['failed_nets'] == []
    assert metrics['iterations'] == 777
    assert metrics['vias'] == 42


def test_a_net_the_reconciliation_broke_is_reported_and_weighed():
    """A coverage-gate net has no routed result, so its pads never reach
    multipoint_pads_total and it would otherwise weigh zero: the loop would
    read failures=0 on a board shipping disconnected copper and stop."""
    metrics, _ = _metrics_for(LOG_RECON_BROKE)
    assert metrics['failed_nets'] == ['/GND_RET'], metrics['failed_nets']
    assert '/A' not in metrics['failed_nets']
    assert metrics['failures'] >= 1, metrics


def test_aborted_reconciliation_falls_back_to_the_first_summary():
    """The sub-run printed a summary and then died, so the board on disk is
    still the first pass's. Failure state falls back; effort still sums."""
    metrics, _ = _metrics_for(LOG_RECON_ABORTED)
    assert metrics['failures'] == 2, metrics
    assert set(metrics['failed_nets']) == {'/A', '/B'}, metrics['failed_nets']
    assert metrics['iterations'] == 530, metrics


def test_reconciliation_without_a_summary_degrades_to_the_first():
    """The sub-run can return before printing anything, e.g. when every
    retried net turns out to be connected already."""
    metrics, _ = _metrics_for(LOG_RECON_NO_SUMMARY)
    assert metrics['failures'] == 1, metrics
    assert metrics['failed_nets'] == ['/A']
    assert metrics['iterations'] == 300


def test_merge_returns_none_without_a_summary():
    assert prl.merge_route_summaries('') is None
    assert prl.merge_route_summaries('Traceback (most recent call last):\n') \
        is None


# --- how route.py is invoked (issue #458 drive-by) ---------------------

def test_route_py_is_invoked_by_absolute_path_in_utf8_mode():
    """A bare relative 'route.py' only resolved from the repo root, and the
    failure surfaced as "produced no JSON_SUMMARY" rather than the real
    error."""
    _, calls = _metrics_for(LOG_ONE_PASS)
    cmd = calls[-1]
    assert cmd[1:3] == ['-X', 'utf8'], cmd
    assert os.path.isabs(cmd[3]), cmd
    assert cmd[3] == os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'py_router', 'route.py'), cmd


def test_nonzero_exit_raises_and_names_the_real_error():
    """route.py exits 0 even when nets fail, so a non-zero code is a crash or
    a bad invocation. It used to be swallowed."""
    try:
        _metrics_for("No nets matched the given patterns!\n", rc=2)
    except RuntimeError as exc:
        assert 'exited 2' in str(exc), exc
        assert 'No nets matched' in str(exc), exc
    else:
        raise AssertionError("a non-zero exit code must not be swallowed")


def test_missing_summary_raises_with_the_log_tail():
    try:
        _metrics_for("Traceback (most recent call last):\nKeyError: 'nets'\n")
    except RuntimeError as exc:
        assert 'JSON_SUMMARY' in str(exc), exc
        assert "KeyError: 'nets'" in str(exc), exc
    else:
        raise AssertionError("a log with no summary must raise")


# --- #504: the ratsnest screen ------------------------------------------------

_WORSE = {'before': {'crossings': 100, 'hpwl': 1000.0, 'length': 1000.0,
                     'total': 1.0},
          'after': {'crossings': 130, 'hpwl': 1000.0, 'length': 1000.0,
                    'total': 0.9}}
_BETTER = {'before': {'crossings': 100, 'hpwl': 1000.0, 'length': 1000.0,
                      'total': 1.0},
           'after': {'crossings': 80, 'hpwl': 900.0, 'length': 900.0,
                     'total': 0.5}}


def test_screen_skips_the_routing_run_on_a_regressed_candidate():
    """The whole point of #504 part 2: routing is minutes per round, so a
    candidate whose ratsnest clearly got worse must not be paid for."""
    routed = []
    _run_loop(['--ratsnest-screen', '5'], rounds=3,
              quench_metrics=_WORSE, route_calls=routed)
    # round 0 always routes the initial placement; every candidate is screened.
    assert len(routed) == 1,         f"expected only the round-0 route, got {len(routed)} routing runs"


def test_screen_off_by_default_routes_every_candidate():
    routed = []
    _run_loop([], rounds=3, quench_metrics=_WORSE, route_calls=routed)
    assert len(routed) == 4,         f"screen must be opt-in: expected 1 + 3 routing runs, got {len(routed)}"


def test_screen_lets_an_improving_candidate_through():
    routed = []
    _run_loop(['--ratsnest-screen', '5'], rounds=3,
              quench_metrics=_BETTER, route_calls=routed)
    assert len(routed) == 4,         f"an improving candidate must still be routed, got {len(routed)}"


def test_screen_is_inert_when_quench_reports_nothing():
    """A fake/old quench that fills no metrics_out must never cause a skip."""
    routed = []
    _run_loop(['--ratsnest-screen', '5'], rounds=2,
              quench_metrics=None, route_calls=routed)
    assert len(routed) == 3, f"got {len(routed)}"


def test_loop_passes_metrics_out_to_quench():
    calls = _run_loop([], rounds=1)
    assert 'metrics_out' in calls[0],         "the loop must ask quench for its ratsnest numbers"
    assert isinstance(calls[0]['metrics_out'], dict)


def test_negative_screen_is_rejected():
    try:
        _run_loop(['--ratsnest-screen', '-1'], rounds=1)
    except SystemExit as e:
        assert e.code == 2, f"argparse should exit 2, got {e.code}"
        return
    raise AssertionError("--ratsnest-screen -1 must be rejected")





def test_target_nets_gives_the_loop_something_to_move_on_a_clean_board():
    """The loop's move candidates come from `failed_single` + `failed_multipoint`.
    So a board where EVERY NET ROUTES but a spec clause is violated -- a maximum
    length, a via ban, a required width -- hands it an empty target list and it
    does nothing at all. That is not a bad move, it is no move, and it is why a
    run whose only blocker was a length clause never re-placed anything.

    `--target-nets` supplies the targets; `--accept-cmd` supplies the gradient.
    Neither alone lets the loop chase a requirement the router is happy with."""
    import place_route_loop as L
    clean = {'failed_single': [], 'failed_multipoint': [],
             'multipoint_pads_total': 10, 'multipoint_pads_connected': 10}
    assert L.metrics_from_summary(dict(clean))['failed_nets'] == [],         "a clean board must have no targets without the flag"
    m = L.metrics_from_summary(dict(clean), extra_targets=['QSPI_SD0', 'QSPI_SD3'])
    assert m['failed_nets'] == ['QSPI_SD0', 'QSPI_SD3']
    # The failure COUNT must not move: better() compares failures then
    # iterations, and inflating it would make every round look like a
    # regression against round 0.
    assert m['failures'] == 0, "targets must not be counted as failures"
    # And a named net that DID fail must not be duplicated.
    m2 = L.metrics_from_summary(
        {'failed_single': ['QSPI_SD0'], 'failed_multipoint': [],
         'multipoint_pads_total': 0, 'multipoint_pads_connected': 0},
        extra_targets=['QSPI_SD0', 'QSPI_SD3'])
    assert m2['failed_nets'] == ['QSPI_SD0', 'QSPI_SD3'], m2['failed_nets']
    print("  PASS: --target-nets seeds the move set, leaves the failure count alone")


TESTS = [
    test_swap_cap_held_while_displacement_widens,
    test_swap_cap_held_when_quench_finds_nothing,
    test_swap_cap_flag_overrides_the_base,
    test_rotate_and_swap_flags_reach_quench,
    test_kwargs_match_the_real_quench_signature,
    test_swap_cap_above_displacement_is_rejected,
    test_reconciliation_recoveries_are_not_counted_as_failures,
    test_effort_counters_are_summed_across_passes,
    test_blockers_are_unioned_across_both_passes,
    test_pad_pairs_keep_the_whole_board_denominator,
    test_pad_pairs_survive_a_sub_run_without_the_keys,
    test_empty_structured_blockers_do_not_regress_to_the_regex,
    test_blockers_filtered_to_still_failed_when_sub_run_omits_the_key,
    test_single_summary_run_is_unchanged,
    test_a_net_the_reconciliation_broke_is_reported_and_weighed,
    test_aborted_reconciliation_falls_back_to_the_first_summary,
    test_reconciliation_without_a_summary_degrades_to_the_first,
    test_merge_returns_none_without_a_summary,
    test_route_py_is_invoked_by_absolute_path_in_utf8_mode,
    test_nonzero_exit_raises_and_names_the_real_error,
    test_missing_summary_raises_with_the_log_tail,
    test_screen_skips_the_routing_run_on_a_regressed_candidate,
    test_screen_off_by_default_routes_every_candidate,
    test_screen_lets_an_improving_candidate_through,
    test_screen_is_inert_when_quench_reports_nothing,
    test_loop_passes_metrics_out_to_quench,
    test_negative_screen_is_rejected,
    # Defined below this list and never registered (#876):
    test_target_nets_gives_the_loop_something_to_move_on_a_clean_board,
]


if __name__ == '__main__':
    for fn in TESTS:
        fn()
    print("ALL PASS")
