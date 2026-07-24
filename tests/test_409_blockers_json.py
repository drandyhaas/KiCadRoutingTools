#!/usr/bin/env python3
"""Issue #409: JSON_SUMMARY carries structured frontier-blocking attribution.

Scenario A (emit): single routing layer, a walled ROOM around the only gap in
a WALL at x=5. BLOCKER (in scope, routed first) takes the trivial vertical
path through the gap; X (left pad -> right pad at y=5) must cross x=5 but
BLOCKER's fresh track plus the WALL end-caps close the slot. With
max_rip_up_count=0 the ladder never fires, so X fails and the run's LAST
frontier analysis for X attributes BLOCKER (in routed_net_paths; the WALL is
static copper and is not frontier-attributable). Asserts the additive
'blockers' key: exactly one entry, net X, stage single_ended, blocked_by
BLOCKER only, and the same list attached to return_results (GUI parity).

Scenario B (staleness): same board, default rip-up. X fails first (recording
BLOCKER), the ladder rips BLOCKER, X routes through the vacated gap;
BLOCKER's reroute then fails un-attributably (X excluded via rip ancestry,
WALL static). The invariant: a net that ULTIMATELY ROUTED never appears in
'blockers' -- last-wins records are filtered to the final failed set.

    python3 tests/test_409_blockers_json.py
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
from synth import make_net, make_pad, make_pcb, make_seg

BLOCKER, X, WALL = 1, 2, 3

BLOCKED_BY_KEYS = {'net', 'blocked_count', 'unique_cells', 'track_cells',
                   'via_cells', 'near_target_cells', 'near_source_cells'}

CHECKS = []


def check(name, ok):
    CHECKS.append((name, ok))
    print(f"  {'PASS' if ok else 'FAIL'}: {name}")


def _board():
    """One routing layer. WALL splits the board at x=5 except a gap
    y in [4.2, 5.8]; around the gap a walled ROOM (openings only at the
    left/right slots at y~5, x=4 and x=6). BLOCKER's pads sit just above and
    below the gap centre; its routed track will span the slot. X (left pad ->
    right pad at y=5) then has no way across (single routing layer)."""
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 10.0, 10.0))
    pads = {
        BLOCKER: [make_pad(BLOCKER, 5.0, 4.5, ref='U3', num='1',
                           net_name='BLOCKER', size_x=0.3, size_y=0.3),
                  make_pad(BLOCKER, 5.0, 5.5, ref='U3', num='2',
                           net_name='BLOCKER', size_x=0.3, size_y=0.3)],
        X: [make_pad(X, 1.0, 5.0, ref='U1', num='1', net_name='X',
                     size_x=0.3, size_y=0.3),
            make_pad(X, 9.0, 5.0, ref='U2', num='1', net_name='X',
                     size_x=0.3, size_y=0.3)],
        WALL: [],
    }
    segs = [
        # WALL copper closing the rest of the x=5 line (static: not in scope,
        # never routed this run, so never frontier-attributable).
        make_seg(5.0, 0.0, 5.0, 4.2, net_id=WALL, width=0.3),
        make_seg(5.0, 5.8, 5.0, 10.0, net_id=WALL, width=0.3),
        # Room around the gap: top/bottom sealed, left/right walls open only
        # at a narrow slot around y=5.
        make_seg(4.0, 4.0, 6.0, 4.0, net_id=WALL, width=0.3),   # bottom
        make_seg(4.0, 6.0, 6.0, 6.0, net_id=WALL, width=0.3),   # top
        make_seg(4.0, 4.0, 4.0, 4.6, net_id=WALL, width=0.3),   # left lower
        make_seg(4.0, 5.4, 4.0, 6.0, net_id=WALL, width=0.3),   # left upper
        make_seg(6.0, 4.0, 6.0, 4.6, net_id=WALL, width=0.3),   # right lower
        make_seg(6.0, 5.4, 6.0, 6.0, net_id=WALL, width=0.3),   # right upper
    ]
    return make_pcb(
        nets={BLOCKER: make_net(BLOCKER, 'BLOCKER'), X: make_net(X, 'X'),
              WALL: make_net(WALL, 'WALL')},
        segments=segs, pads_by_net=pads, board_info=bi)


def _route(pcb, **kwargs):
    from route import batch_route
    buf = io.StringIO()
    with redirect_stdout(buf):
        ok, fail, _t, results_data = batch_route(
            'synthetic', '', ['BLOCKER', 'X'],
            layers=['F.Cu'],
            clearance=0.1, track_width=0.15,
            via_size=0.5, via_drill=0.3, grid_step=0.1,
            ordering_strategy='original',
            final_reconcile=False,
            return_results=True, pcb_data=pcb, **kwargs)
    out = buf.getvalue()
    sys.stdout.write(out)
    m = re.findall(r'JSON_SUMMARY: (\{.*\})', out)
    check("JSON_SUMMARY present", bool(m))
    return (json.loads(m[-1]) if m else {}), results_data, out


def scenario_a():
    print("-" * 60)
    print("Scenario A: emit (max_rip_up_count=0)")
    summary, results_data, out = _route(_board(), max_rip_up_count=0)

    check("BLOCKER routed", 'BLOCKER' in summary.get('routed_single', []))
    check("X in failed_single", 'X' in summary.get('failed_single', []))

    blockers = summary.get('blockers')
    check("'blockers' key present", isinstance(blockers, list))
    blockers = blockers or []
    check("exactly one blockers entry", len(blockers) == 1)
    entry = blockers[0] if blockers else {}
    check("entry net is X", entry.get('net') == 'X')
    check("entry stage is single_ended", entry.get('stage') == 'single_ended')

    blocked_by = entry.get('blocked_by') or []
    check("blocked_by attributes BLOCKER only",
          [b.get('net') for b in blocked_by] == ['BLOCKER'])
    b0 = blocked_by[0] if blocked_by else {}
    check("blocked_by entry has exactly the documented keys",
          set(b0.keys()) == BLOCKED_BY_KEYS)
    counts = [b0.get(k) for k in BLOCKED_BY_KEYS - {'net'}]
    check("all counts are ints >= 0",
          all(isinstance(v, int) and v >= 0 for v in counts))
    check("blocked_count >= unique_cells > 0",
          b0.get('blocked_count', 0) >= b0.get('unique_cells', 0) > 0)

    check("return_results carries the same blockers (GUI parity)",
          results_data.get('blockers') == summary.get('blockers'))


def scenario_b():
    print("-" * 60)
    print("Scenario B: staleness (default rip-up)")
    summary, results_data, out = _route(_board())

    check("X routed after ripping BLOCKER",
          'X' in summary.get('routed_single', []))
    check("BLOCKER in failed_single",
          'BLOCKER' in summary.get('failed_single', []))
    # The staleness invariant: X failed transiently (its analysis recorded
    # BLOCKER) but ultimately routed, so it must NOT appear in 'blockers'.
    # (The key itself may be absent -- BLOCKER's own failure has no
    # attributable blockers -- but we deliberately don't assert absence.)
    nets_reported = [e.get('net') for e in summary.get('blockers', [])]
    check("routed net X never appears in blockers", 'X' not in nets_reported)


def main():
    print("=" * 60)
    print("Issue #409: structured blockers in JSON_SUMMARY")
    print("=" * 60)
    scenario_a()
    scenario_b()
    failed = [n for n, okk in CHECKS if not okk]
    print("-" * 60)
    print(f"{len(CHECKS) - len(failed)}/{len(CHECKS)} checks passed")
    if failed:
        print("FAILED: " + ", ".join(failed))
        sys.exit(1)
    print("ALL PASS")


if __name__ == '__main__':
    main()
