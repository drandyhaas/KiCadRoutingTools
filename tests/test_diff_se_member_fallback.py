#!/usr/bin/env python3
"""Class 2: the diff single-ended fallback must attempt BOTH pair members.

A pair that fails/defers coupled routing used to leave batch_route_diff_pairs
with NEITHER member attempted (only a printed suggestion for a downstream
single-ended pass), and downstream churn could route one member while the
other shipped at zero copper (ulx3s FPDI_D1: FPDI_D1- routed, FPDI_D1+
dropped). run_single_ended_member_fallback now attempts every disconnected
member of every non-coupled pair in-run, and pair_reports carry the
per-member outcome.

Part 1 (end-to-end): an electrically-short pair is deferred by design; the
fallback must route BOTH members and report them per-member.
Part 2 (honesty, stubbed): if one member's route fails, the other must STILL
be attempted, and the failure must be reported per-member -- never silent.

    python3 tests/test_diff_se_member_fallback.py
"""
import io
import json
import os
import re
import sys
from contextlib import redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import BoardInfo
from synth import make_net, make_pad, make_pcb

P, N = 1, 2

CHECKS = []


def check(name, ok):
    CHECKS.append((name, ok))
    print(f"  {'PASS' if ok else 'FAIL'}: {name}")


def _board():
    """A 2-terminal diff pair whose span (~1.2mm) is below the coupled floor,
    so batch_route_diff_pairs defers the whole pair to single-ended."""
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 6.0, 6.0))
    pads = {
        P: [make_pad(P, 2.0, 2.0, ref='U1', num='1', net_name='/D_P',
                     size_x=0.3, size_y=0.3),
            make_pad(P, 3.2, 2.0, ref='J1', num='1', net_name='/D_P',
                     size_x=0.3, size_y=0.3)],
        N: [make_pad(N, 2.0, 2.6, ref='U1', num='2', net_name='/D_N',
                     size_x=0.3, size_y=0.3),
            make_pad(N, 3.2, 2.6, ref='J1', num='2', net_name='/D_N',
                     size_x=0.3, size_y=0.3)],
    }
    return make_pcb(
        nets={P: make_net(P, '/D_P'), N: make_net(N, '/D_N')},
        segments=[], pads_by_net=pads, board_info=bi)


def _run(pcb):
    from route_diff import batch_route_diff_pairs
    buf = io.StringIO()
    with redirect_stdout(buf):
        res = batch_route_diff_pairs(
            'synthetic', '', ['/D_*'],
            layers=['F.Cu', 'B.Cu'],
            clearance=0.1, track_width=0.15, diff_pair_gap=0.15,
            via_size=0.5, via_drill=0.3, grid_step=0.1,
            return_results=True, pcb_data=pcb)
    return res, buf.getvalue()


def test_end_to_end_both_members_routed():
    print("Part 1: deferred pair -> BOTH members single-ended in-run")
    pcb = _board()
    (ok, fail, _t, results_data), out = _run(pcb)
    sys.stdout.write(out)

    m = re.findall(r'JSON_SUMMARY: (\{.*\})', out)
    check("JSON_SUMMARY present", bool(m))
    summary = json.loads(m[-1]) if m else {}
    check("pair deferred to single-ended (not coupled, not failed)",
          '/D' in ' '.join(summary.get('single_ended_diff_pairs', []))
          or summary.get('single_ended_diff_pairs'))

    se = summary.get('single_ended_fallback') or {}
    check("fallback attempted both members",
          set(se.get('attempted', [])) == {'/D_P', '/D_N'})
    check("fallback routed both members",
          set(se.get('routed', [])) == {'/D_P', '/D_N'})

    p_segs = [s for r in results_data['results']
              for s in (r.get('new_segments') or []) if s.net_id == P]
    n_segs = [s for r in results_data['results']
              for s in (r.get('new_segments') or []) if s.net_id == N]
    check("P member has copper in the write model", bool(p_segs))
    check("N member has copper in the write model", bool(n_segs))

    reps = {r['pair']: r for r in summary.get('pair_reports', [])}
    rep = next(iter(reps.values())) if reps else {}
    check("pair report outcome is NOT 'coupled' (fallback flag honored)",
          rep.get('outcome') != 'coupled')
    check("pair report carries per-member se_fallback outcomes",
          rep.get('se_fallback', {}).get('p') == 'routed'
          and rep.get('se_fallback', {}).get('n') == 'routed')
    check("member audit clean after fallback (both connected)",
          rep.get('incomplete_members') == [])
    check("results_data mirrors the fallback tally (GUI parity)",
          set((results_data.get('single_ended_fallback') or {})
              .get('routed', [])) == {'/D_P', '/D_N'})


def test_one_member_failure_is_honest():
    print("\nPart 2: one member fails -> other still attempted, honest report")
    import single_ended_routing
    orig = single_ended_routing.route_net_with_obstacles

    def fail_n(pcb_data, net_id, config, obstacles, **kw):
        if net_id == N:
            return {'failed': True, 'iterations': 0}
        return orig(pcb_data, net_id, config, obstacles, **kw)

    pcb = _board()
    single_ended_routing.route_net_with_obstacles = fail_n
    try:
        (ok, fail, _t, results_data), out = _run(pcb)
    finally:
        single_ended_routing.route_net_with_obstacles = orig
    sys.stdout.write(out)

    m = re.findall(r'JSON_SUMMARY: (\{.*\})', out)
    summary = json.loads(m[-1]) if m else {}
    se = summary.get('single_ended_fallback') or {}
    check("both members attempted despite N's failure",
          set(se.get('attempted', [])) == {'/D_P', '/D_N'})
    check("P routed", se.get('routed') == ['/D_P'])
    check("N failure reported honestly", se.get('failed') == ['/D_N'])
    reps = summary.get('pair_reports', [])
    rep = reps[0] if reps else {}
    check("per-member outcome recorded (p=routed, n=failed)",
          rep.get('se_fallback', {}).get('p') == 'routed'
          and rep.get('se_fallback', {}).get('n') == 'failed')
    check("failed member still listed incomplete (never silent)",
          '/D_N' in (rep.get('incomplete_members') or []))
    n_segs = [s for r in results_data['results']
              for s in (r.get('new_segments') or []) if s.net_id == N]
    check("no phantom N copper", not n_segs)


def main():
    print("=" * 60)
    print("diff single-ended member fallback tests (Class 2)")
    print("=" * 60)
    test_end_to_end_both_members_routed()
    test_one_member_failure_is_honest()
    failed = [n for n, okk in CHECKS if not okk]
    print("-" * 60)
    print(f"{len(CHECKS) - len(failed)}/{len(CHECKS)} checks passed")
    if failed:
        print("FAILED: " + ", ".join(failed))
        sys.exit(1)
    print("ALL PASS")


if __name__ == '__main__':
    main()
