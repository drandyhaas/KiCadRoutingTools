#!/usr/bin/env python3
"""#1069: the merged route tally must not lose a net an earlier pass left broken.

route.py's `--json-out` and JSON_SUMMARY_MIN merge every JSON_SUMMARY one run
prints: pass 1, the plane finalize's repair sub-runs, the reconciliation laps.
Each summary grades only its own scope at its own moment, and the merge used
to take the failure state from the LAST one. Measured on glasgow run34: the
file said 19 failed while check_connected found 40 disconnected nets on disk,
20 of them in no bucket at all. Three defects, one check each:

1. the sticky union of coverage-gate / ripped-open nets dropped a net as soon
   as ANY later summary classified it, even a middle one whose buckets
   last-wins then overwrote (the "case 6" shape);
2. the CLI had TWO summary sinks: route.py is `__main__` there, and the
   finalize reaches its sub-runs through repair_planes' `from route import
   batch_route`, a second module copy whose summaries the log carried and the
   file never saw;
3. nothing graded the shipped board at the end, so no merge could report a
   net no summary had recorded.

The fix grades the shipped board once over every net the run owns
(`route_summary.regrade_record`, printed as JSON_REGRADE) and both merges
apply it. The last check routes a real chain (pour, then a route step whose
in-run plane finalize spawns nested sub-runs) and holds the corpus invariant
from the issue: every net check_connected reports disconnected is in
failed_single, open_single, failed_multipoint or pad_pairs_open -- and the
file equals merge_route_summaries(log).

That chain ends either way, by environment. Where KiCad is installed, the
finalize's oracle legs run and the run breaks three nets while connecting
GND and +3V3. The improvement gate (#600) then REJECTS it and the output is
the input board, on which the plane nets are open (the pour deferred their
pads to this step). By #1173's contract the file then says
`"shipped": "input board"` and its tallies describe the rejected attempt. So
on that path the invariant is held against what the document does name for
the shipped board: the attempt's failures, or a net the attempt would have
connected (`improvement_gate.gained`). Where KiCad is absent the run is kept
and the plain invariant applies.
"""
import importlib.util
import json
import os
import re
import subprocess
import sys
import tempfile

RUN_ALL_TIMEOUT = 1200   # the e2e chain routes for ~4 minutes

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
import route_summary  # noqa: E402
from route_summary import (merge_summaries, merge_route_summaries,  # noqa: E402
                           regrade_record, summary_min)


def _s(scope, routed=(), fs=(), os_=(), fm=(), **extra):
    """A summary dict shaped like route.py's."""
    d = {'scope': scope, 'successful': len(routed),
         'failed': len(fs) + len(os_) + len(fm),
         'routed_single': list(routed), 'failed_single': list(fs),
         'open_single': list(os_),
         'failed_multipoint': [{'net_name': n, 'net_id': i,
                                'failed_pads': [{'x': 0, 'y': 0,
                                                 'component_ref': 'U1',
                                                 'pad_number': '?'}]}
                               for i, n in enumerate(fm)],
         'multipoint_pads_total': 0, 'multipoint_pads_connected': 0,
         'total_iterations': 10, 'total_vias': 1, 'total_time': 1.0}
    d.update(extra)
    return d


def _g(pads=2, broken=True, copper=True, k=1):
    return {'pads': pads, 'broken': broken, 'copper': copper,
            'failed_pads': [{'x': 0, 'y': 0, 'component_ref': 'U1',
                             'pad_number': '?'}] * (k if broken else 0)}


def test_case6_gate_net_reclassified_by_a_middle_summary():
    """Gate net G in sub 1, failed_single in middle sub 2, and a last lap
    that never looks at G: G used to land in NO bucket (deficit 0)."""
    sums = [_s('run', ['X'], ['A']),
            _s('reconciliation-subset', fm=['G'], coverage_gate_nets=['G']),
            _s('reconciliation-subset', fs=['G']),
            _s('reconciliation-subset', fs=['A'])]
    m = merge_summaries(sums)
    assert m['coverage_gate_nets'] == ['G'], m.get('coverage_gate_nets')
    deficit = m['multipoint_pads_total'] - m['multipoint_pads_connected']
    count = len(m['failed_single']) + len(m['open_single']) + deficit
    assert count == 2, f"A and G must both count, got {count}: {m}"
    # ...but a flag the FINAL summary re-classifies is carried by last-wins,
    # and one a later summary ROUTED is recovered: neither may double-count.
    last_owns = merge_summaries(sums[:3] + [_s('reconciliation-subset',
                                                fs=['A', 'G'])])
    assert not last_owns.get('coverage_gate_nets'), last_owns
    recovered = merge_summaries(sums[:2] + [_s('reconciliation-subset',
                                               routed=['G'])] + sums[3:])
    assert not recovered.get('coverage_gate_nets'), recovered
    print("  PASS: a flag survives a middle summary's classification")


def test_issue_literal_disjoint_case():
    """The issue's unit: run(A open), subset(B open), subset(C open)."""
    sums = [_s('run', ['X'], ['A']), _s('reconciliation-subset', fs=['B']),
            _s('reconciliation-subset', fs=['C'])]
    # Without a re-grade the merge can only infer. B, a MIDDLE summary's
    # failure no later summary looked at, is now carried; A is not, because a
    # lap retries every net pass 1 left failing, so a lap that did not
    # classify A found it connected.
    m = merge_summaries(sums)
    assert sorted(m['failed_single']) == ['B', 'C'], m['failed_single']
    assert m['scope'] == 'merged'
    # With the re-grade the board decides, and A, B and C are all there.
    rg = regrade_record(sums, {'A': _g(copper=False), 'B': _g(copper=False),
                               'C': _g(copper=False), 'X': _g(broken=False)},
                        ['X', 'A'], board='file')
    m = merge_summaries(sums, regrade=rg)
    assert sorted(m['failed_single']) == ['A', 'B', 'C'], m['failed_single']
    assert m['scope'] == 'merged'
    # `successful` counts the routing scope (X, A); `failed` every net the
    # run ships broken, the laps' B and C included (#1215).
    assert (m['successful'], m['failed']) == (1, 3), (
        "successful counts the routing scope, failed what ships broken")
    mn = summary_min(m)
    assert mn['failed_single'] == ['A', 'B', 'C'] and mn['routed'] == 1
    print("  PASS: A, B and C present, scope merged")


def test_scope_is_stamped_merged():
    one = _s('run', ['X'])
    assert merge_summaries([one])['scope'] == 'run', (
        "a single summary is its own word and keeps its scope")
    two = merge_summaries([one, _s('reconciliation-subset', ['Y'])])
    assert two['scope'] == 'merged', two['scope']
    rg = regrade_record([one], {'X': _g(broken=False)}, ['X'], board='file')
    assert merge_summaries([one], regrade=rg)['scope'] == 'merged'
    print("  PASS: scope == 'merged' on every merged document")


def test_regrade_keeps_bucket_meanings():
    """Each broken net lands in the bucket route.py's definitions give it,
    so failed_single + open_single + deficit weighs a two-pad net 1 and a
    multipoint net by its missing pads -- never twice."""
    sums = [_s('run', ['R', 'L'], ['F'], ['O'], ['O', 'M']),
            _s('reconciliation-subset', ['F'])]
    grades = {
        'F': _g(copper=True),                  # last word 'routed', 2 pads
        'O': _g(copper=True),                  # last word open_single
        'M': _g(pads=5, k=2),                  # multipoint, 2 pads off
        'R': _g(broken=False),                 # fine
        'L': _g(pads=4, k=1, copper=True),     # routed then LOST, 4 pads
        'V': _g(copper=False),                 # collateral, no copper
    }
    rg = regrade_record(sums, grades, ['R', 'L', 'F', 'O', 'M'],
                        board='file', disturbed_only=['V'])
    assert rg['failed_single'] == ['V'], rg['failed_single']
    assert rg['open_single'] == ['F', 'O'], rg['open_single']
    assert sorted(d['net_name'] for d in rg['failed_multipoint']) == \
        ['F', 'L', 'M', 'O']
    assert rg['multipoint_pads_total'] == 9, rg      # M 5 + L 4
    assert rg['multipoint_pads_connected'] == 6, rg  # M 3 + L 3
    assert rg['unowned_broken'] == ['V'], rg['unowned_broken']
    assert rg['recovered'] == [], rg['recovered']
    # #1215: `failed` counts every net the run ships broken, V (collateral,
    # outside the scope) included -- not len(scope) - successful.
    assert (rg['successful'], rg['failed']) == (1, 5), rg
    m = merge_summaries(sums, regrade=rg)
    deficit = m['multipoint_pads_total'] - m['multipoint_pads_connected']
    assert len(m['failed_single']) + len(m['open_single']) + deficit == 6
    print("  PASS: bucket meanings kept; no net weighed twice")


def test_log_merge_reads_only_this_runs_regrade():
    rg = regrade_record([_s('run', ['X'])], {'X': _g(copper=False)}, ['X'],
                        board='file')
    line = 'JSON_REGRADE: ' + json.dumps(rg)
    s = 'JSON_SUMMARY: ' + json.dumps(_s('run', ['X']))
    after = merge_route_summaries('\n'.join([s, line]))
    assert after['failed_single'] == ['X'] and 'regrade' in after
    before = merge_route_summaries('\n'.join([line, s]))
    assert before['failed_single'] == [] and 'regrade' not in before, (
        "a re-grade printed BEFORE the last summary belongs to another run")
    print("  PASS: the log merge applies the run's own re-grade only")


def test_nested_import_shares_the_sink():
    """The CLI runs route.py as __main__; repair_planes imports it as
    `route`. Both copies must append to ONE list."""
    import route
    path = os.path.join(ROOT, 'py_router', 'route.py')
    spec = importlib.util.spec_from_file_location('route_as_main_1069', path)
    twin = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(twin)
    assert twin is not route
    for name, shared in (('_SUMMARY_SINK', route_summary.SUMMARY_SINK),
                         ('_RECONCILE_RAISED', route_summary.RECONCILE_RAISED),
                         ('_FINAL_REGRADE', route_summary.FINAL_REGRADE)):
        assert getattr(route, name) is shared, name
        assert getattr(twin, name) is shared, (
            f"{name}: a second copy of route.py has its own list again")
    route_summary.reset_run_state()
    twin._SUMMARY_SINK.append({'probe': 1})
    try:
        assert route._SUMMARY_SINK == [{'probe': 1}]
    finally:
        route_summary.reset_run_state()
    print("  PASS: both module copies share one summary sink")


def _disconnected(pcb):
    out = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_router', 'check_connected.py'), pcb],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=ROOT).stdout
    assert re.search(r'ALL NETS FULLY CONNECTED|FOUND \d+ ISSUES', out), (
        "check_connected produced no verdict:\n" + out[-2000:])
    unrouted = re.findall(r'^    (\S.*?) \(\d+ pads\)$', out, re.M)
    split = re.findall(r'^  (\S.*?) \(net \d+\):$', out, re.M)
    return set(unrouted) | set(split)


def test_e2e_invariant_with_in_run_plane_finalize():
    board = os.path.join(ROOT, 'kicad_files',
                         'rp2350_fpga_eensy_prePlane.kicad_pcb')
    if not os.path.isfile(board):
        raise AssertionError(f"fixture missing: {board}")
    py = [sys.executable, '-X', 'utf8']
    R = lambda s: os.path.join(ROOT, 'py_router', s)  # noqa: E731
    with tempfile.TemporaryDirectory() as td:
        b0 = os.path.join(td, 'b0.kicad_pcb')
        planes = os.path.join(td, 'planes.kicad_pcb')
        out = os.path.join(td, 'out.kicad_pcb')
        js = os.path.join(td, 'out.json')
        # The recorded #562 chain (tests/gui_parity/test_gui_livechain_rp2350):
        # a bare pour, then ONE route step carrying the plane nets, whose
        # in-run finalize spawns the nested repair sub-runs.
        steps = [
            py + [R('copy_board.py'), board, b0],
            py + [R('route_planes.py'), b0, planes, '--nets', 'GND', '+3V3',
                  '--plane-layers', 'In1.Cu', 'In4.Cu', '--via-size', '0.45',
                  '--via-drill', '0.2', '--track-width', '0.09',
                  '--clearance', '0.10', '--hole-to-hole-clearance', '0.2',
                  '--grid-step', '0.05', '--power-nets', 'VIN',
                  '--power-nets-widths', '0.3'],
            py + [R('route.py'), planes, out, '--nets', '+1V1',
                  '/T8F49I2X/PIN.5', 'GND', '+3V3', '--layers', 'F.Cu',
                  'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'B.Cu',
                  '--no-bga-zones', '--clearance', '0.09', '--track-width',
                  '0.0762', '--via-size', '0.25', '--via-drill', '0.15',
                  '--hole-to-hole-clearance', '0.2', '--grid-step', '0.025',
                  '--max-ripup', '10', '--max-iterations', '1000000',
                  '--json-out', js],
        ]
        log = ''
        for cmd in steps:
            r = subprocess.run(cmd, capture_output=True, text=True,
                               encoding='utf-8', errors='replace', cwd=td)
            assert r.returncode == 0, (
                f"step failed rc={r.returncode}: {cmd[3]}\n"
                + (r.stdout or '')[-2000:] + (r.stderr or '')[-2000:])
            log = r.stdout
        assert os.path.isfile(js), "--json-out wrote no file"
        doc = json.load(open(js, encoding='utf-8'))
        raw = [json.loads(x) for x in route_summary.SUMMARY_RE.findall(log)]
        assert len(raw) >= 2, (
            f"the chain printed {len(raw)} JSON_SUMMARY line(s); it no longer "
            f"exercises nested sub-runs, so this check tests nothing")
        assert [x.get('scope') for x in raw].count('run') == 1, (
            "only the outermost pass may call itself scope='run': "
            + str([x.get('scope') for x in raw]))
        assert route_summary.REGRADE_RE.search(log), "no JSON_REGRADE line"
        # One document (#830): every summary the log carries reached the file.
        assert doc == merge_route_summaries(log), (
            "--json-out and merge_route_summaries(log) disagree")
        assert doc['total_iterations'] == sum(
            x.get('total_iterations', 0) for x in raw), (
            "--json-out did not see every summary the log printed")
        assert doc['scope'] == 'merged', doc.get('scope')
        # The corpus invariant from the issue.
        buckets = (set(doc.get('failed_single') or [])
                   | set(doc.get('open_single') or [])
                   | {d['net_name'] for d in doc.get('failed_multipoint') or []}
                   | {p['net'] for p in doc.get('pad_pairs_open') or []})
        named, path = buckets, 'run kept'
        if doc.get('shipped') == 'input board':
            # #1173: the gate rejected the run and shipped the input board,
            # so the tallies describe the rejected attempt. The document must
            # say so, the output must BE the input, and every net open on it
            # must still be named: as an attempt failure, or as a net the
            # attempt connected and the revert gave back.
            gate = doc.get('improvement_gate') or {}
            assert gate.get('verdict') == 'reject', (
                f"shipped 'input board' without a rejecting gate: {gate}")
            assert 'rejected attempt' in (doc.get('shipped_note') or ''), (
                "a reverted run's file does not say its tallies are the "
                "rejected attempt's")
            with open(planes, 'rb') as a, open(out, 'rb') as b:
                assert a.read() == b.read(), (
                    "the file says the input board shipped, and the output "
                    "differs from the input")
            named = buckets | set(gate.get('gained') or [])
            path = f"gate reverted, gained {sorted(gate.get('gained') or [])}"
        missing = _disconnected(out) - named
        assert not missing, (
            f"check_connected reports {sorted(missing)} disconnected, and "
            f"--json-out names them nowhere ({path})")
        print(f"  PASS: {len(raw)} summaries, file == log merge, "
              f"disconnected nets all reported ({path}; {sorted(named)})")


if __name__ == '__main__':
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        print(f"--- {fn.__name__}")
        fn()
    print("ALL PASS")
