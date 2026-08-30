#!/usr/bin/env python3
"""`route.py --json-out` writes the MERGED tally, once, even when the
reconciliation sub-run fires.

Two things can go wrong and both are silent:

1. The sub-run overwrites the parent's file. `batch_route` forwards its whole
   parameter snapshot to the reconciliation self-invoke (`dict(locals())`), so a
   new parameter is auto-forwarded unless it is explicitly popped -- and the
   sub-run would then write ITS summary, which is scoped to the retried nets, over
   the parent's whole-board one. A reader gets `pad_pairs_total: 3` for a 58-pair
   board and never knows.
2. The file carries an unmerged summary. Scraping either emission alone is wrong
   in a different direction each time: the first counts every reconciliation
   recovery as a failure, the second narrows the whole-board denominator.

The merge itself is `route_summary.merge_summaries`, shared with
place_route_loop so the two cannot drift.
"""
import inspect
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
from route_summary import merge_summaries, merge_route_summaries  # noqa: E402


def test_json_out_is_popped_from_the_reconcile_snapshot():
    """The one-line guard that stops the sub-run clobbering the parent's file."""
    import route
    src = inspect.getsource(route.batch_route)
    m = re.search(r"for _k in \(([^)]*)\):", src, re.S)
    assert m, "batch_route no longer pops anything from its reconcile snapshot"
    popped = m.group(1)
    assert "'json_out'" in popped, (
        "json_out must be popped from _reconcile_kwargs, or the reconciliation "
        "sub-run writes its own reconcile-subset summary over the parent's file")
    assert 'json_out' in inspect.signature(route.batch_route).parameters
    print("  PASS: json_out is a batch_route param and is popped before forwarding")


def test_merge_keeps_whole_board_denominator_and_sums_effort():
    """The two-summary reduction, which is what --json-out writes."""
    first = {'failed_single': ['A', 'B'], 'failed_multipoint': [],
             'multipoint_pads_total': 10, 'multipoint_pads_connected': 6,
             'total_iterations': 1000, 'total_vias': 4, 'total_time': 2.0,
             'pad_pairs_total': 58, 'pad_pairs_connected': 50,
             'pad_pairs_open': [{'net': 'A', 'pairs_total': 4, 'pairs_connected': 0},
                                {'net': 'B', 'pairs_total': 4, 'pairs_connected': 0}],
             'blockers': [{'net': 'A', 'blocked_by': [{'net': 'Z'}]},
                          {'net': 'B', 'blocked_by': [{'net': 'Y'}]}]}
    # The sub-run recovered A, and is scoped to the retry set only.
    sub = {'failed_single': ['B'], 'failed_multipoint': [],
           'multipoint_pads_total': 4, 'multipoint_pads_connected': 2,
           'total_iterations': 250, 'total_vias': 1, 'total_time': 0.5,
           'pad_pairs_total': 8, 'pad_pairs_connected': 4,
           'pad_pairs_open': [{'net': 'B', 'pairs_total': 4, 'pairs_connected': 0}]}

    m = merge_summaries([first, sub])
    assert m['failed_single'] == ['B'], "failure state is last-wins (A recovered)"
    assert m['total_iterations'] == 1250, "effort SUMS across passes"
    assert m['total_vias'] == 5 and m['total_time'] == 2.5
    assert m['pad_pairs_total'] == 58, (
        "the denominator must stay whole-board, not narrow to the retry subset")
    assert m['pad_pairs_connected'] == 54, (
        "connected = whole-board total minus what is STILL open (B's 4)")
    print("  PASS: last-wins state, summed effort, whole-board denominator")


def test_aborted_reconciliation_falls_back_to_what_is_on_disk():
    """A sub-run that raised AFTER printing claims recoveries the board write
    may never have committed."""
    first = {'failed_single': ['A'], 'total_iterations': 10}
    sub = {'failed_single': [], 'total_iterations': 5}
    assert merge_summaries([first, sub], aborted=False)['failed_single'] == []
    assert merge_summaries([first, sub], aborted=True)['failed_single'] == ['A'], (
        "an aborted reconciliation must not advertise its recoveries")
    print("  PASS: aborted reconciliation falls back to pass 1")


def test_single_summary_degrades_unchanged():
    one = {'failed_single': [], 'total_iterations': 7, 'pad_pairs_total': 3}
    assert merge_summaries([one]) == dict(one, total_iterations=7,
                                          total_vias=0, total_time=0)
    assert merge_summaries([]) is None
    print("  PASS: single-summary and empty cases")


def test_disturbed_unowned_nets_survive_the_merge():
    """#622 yw1: reconcile rip-victims OUTSIDE the sub-run's --nets scope
    shipped open (one at ZERO copper) while the merged MIN said deficit 0 --
    a later, narrower sub-run's summary carried none of the disclosure keys
    and last-wins erased them. The victims must stay counted unless a later
    pass CLASSIFIES them (routed = recovered; failed/open = owned there)."""
    main = {'routed_single': ['X', 'V1', 'V2'], 'failed_single': ['Y'],
            'total_iterations': 10}
    reconcile = {'routed_single': ['Y'], 'failed_single': [],
                 'coverage_gate_nets': ['V1'],
                 'ripped_open_uncounted': ['V1', 'V2'],
                 'terminal_restores': {'V1': 'stub', 'V2': 'stub'},
                 'total_iterations': 5}
    narrow_final = {'routed_single': [], 'failed_single': ['Z'],
                    'total_iterations': 2}
    m = merge_summaries([main, reconcile, narrow_final])
    assert sorted(m['coverage_gate_nets']) == ['V1', 'V2'], m.get(
        'coverage_gate_nets')
    assert m['terminal_restores'] == {'V1': 'stub', 'V2': 'stub'}
    assert m['multipoint_pads_total'] == 2, (
        "each unowned victim must widen the deficit denominator by 1")
    # a victim a LATER pass re-routed is recovered, not broken
    recovered = merge_summaries(
        [main, reconcile, dict(narrow_final, routed_single=['V2'])])
    assert sorted(recovered['coverage_gate_nets']) == ['V1']
    assert recovered['terminal_restores'] == {'V1': 'stub'}
    # SAME-pass supersession, single-summary form (yv3: mark stub yet the
    # pass-end routed_single, re-derived from the union-find, has the net)
    solo = {'routed_single': ['W'], 'failed_single': [],
            'terminal_restores': {'W': 'stub'}, 'total_iterations': 1}
    assert merge_summaries([solo])['terminal_restores'] == {}
    print("  PASS: disturbed-but-unowned nets survive the merge")


def test_end_to_end_file_equals_merged_stdout():
    """The real thing: run route.py on an in-repo fixture and compare."""
    board = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
    if not os.path.isfile(board):
        print("  SKIP: fixture missing")
        return
    with tempfile.TemporaryDirectory() as td:
        js = os.path.join(td, 's.json')
        out = os.path.join(td, 's.kicad_pcb')
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_router', 'route.py'), board, out,
                            '--nets', 'GND', '--json-out', js],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=ROOT)
        assert r.returncode == 0, r.stdout[-2000:]
        assert os.path.isfile(js), "--json-out wrote no file"
        from_file = json.load(open(js, encoding='utf-8'))
        from_stdout = merge_route_summaries(r.stdout)
        assert from_file == from_stdout, (
            "the file and the merged stdout disagree:\n"
            f"  file keys: {sorted(from_file)}\n  stdout keys: {sorted(from_stdout or {})}")
    print("  PASS: --json-out == merged(stdout) on a real run")


if __name__ == '__main__':
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        print(f"--- {fn.__name__}")
        fn()
    print("ALL PASS")
