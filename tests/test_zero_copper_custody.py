#!/usr/bin/env python3
"""T5 zero-copper custody on the route.py front (ulx3s GN8/GP2/GN22 class).

Scenario: a pre-existing, ALREADY-CONNECTED net (VICTIM) is made rippable via
rip_existing_nets. The in-scope net X can only route through VICTIM's corridor,
so the rip-up ladder rips VICTIM and X routes through the vacated gap. VICTIM's
reroute then has nowhere to go (single routing layer, X's new track crosses its
only path), so pre-fix it shipped at ZERO copper with NO failure record:

  - not in the run's scope (not in failed_single / failed_multipoint),
  - its ripped original copper stale-stripped from the output (#220),
  - the authoritative sweep graded it on the STRIPPED ghost copper.

Post-fix invariants checked here:
  1. run_casualty_reconcile (now consumed by batch_route) attempts custody:
     restore-first, then depth-1 reroute, then piece-level partial restore --
     honest 'unrecovered' when nothing safe fits (X's track owns the corridor).
  2. The COVERAGE GATE reports the disturbed, still-broken net as a
     failed_multipoint entry (owned, retryable) -- never a silent zero.
  3. The write model stays DRC-safe: no restored VICTIM copper crossing X.

    python3 tests/test_zero_copper_custody.py
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

X, VICTIM, WALL = 1, 2, 3

CHECKS = []


def check(name, ok):
    CHECKS.append((name, ok))
    print(f"  {'PASS' if ok else 'FAIL'}: {name}")


def _board():
    """One routing layer. A WALL splits the board at x=5 except a gap
    y in [4.2, 5.8]; around the gap a walled ROOM (openings only at the
    left/right slots at y~5, x=4 and x=6) contains VICTIM, pre-routed
    straight through the gap. X (left pad -> right pad at y=5) can only
    cross by ripping VICTIM -- and once X's track spans the room
    wall-to-wall, VICTIM has no legal path left (single routing layer)."""
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 10.0, 10.0))
    pads = {
        X: [make_pad(X, 1.0, 5.0, ref='U1', num='1', net_name='X',
                     size_x=0.3, size_y=0.3),
            make_pad(X, 9.0, 5.0, ref='U2', num='1', net_name='X',
                     size_x=0.3, size_y=0.3)],
        VICTIM: [make_pad(VICTIM, 5.0, 4.5, ref='U3', num='1',
                          net_name='VICTIM', size_x=0.3, size_y=0.3),
                 make_pad(VICTIM, 5.0, 5.5, ref='U3', num='2',
                          net_name='VICTIM', size_x=0.3, size_y=0.3)],
        WALL: [],
    }
    segs = [
        # VICTIM pre-routed (connected): its pads bridged through the gap.
        make_seg(5.0, 4.5, 5.0, 5.5, net_id=VICTIM, width=0.15),
        # WALL copper closing the rest of the x=5 line (unrippable: WALL is
        # not in rip_existing_nets and not in scope).
        make_seg(5.0, 0.0, 5.0, 4.2, net_id=WALL, width=0.3),
        make_seg(5.0, 5.8, 5.0, 10.0, net_id=WALL, width=0.3),
        # Room around the gap: top/bottom sealed, left/right walls open only
        # at a narrow slot around y=5 that X's track will fill.
        make_seg(4.0, 4.0, 6.0, 4.0, net_id=WALL, width=0.3),   # bottom
        make_seg(4.0, 6.0, 6.0, 6.0, net_id=WALL, width=0.3),   # top
        make_seg(4.0, 4.0, 4.0, 4.6, net_id=WALL, width=0.3),   # left lower
        make_seg(4.0, 5.4, 4.0, 6.0, net_id=WALL, width=0.3),   # left upper
        make_seg(6.0, 4.0, 6.0, 4.6, net_id=WALL, width=0.3),   # right lower
        make_seg(6.0, 5.4, 6.0, 6.0, net_id=WALL, width=0.3),   # right upper
    ]
    return make_pcb(
        nets={X: make_net(X, 'X'), VICTIM: make_net(VICTIM, 'VICTIM'),
              WALL: make_net(WALL, 'WALL')},
        segments=segs, pads_by_net=pads, board_info=bi)


def main():
    print("=" * 60)
    print("T5 zero-copper custody test (route.py front)")
    print("=" * 60)
    from route import batch_route

    pcb = _board()
    victim_orig_seg = [s for s in pcb.segments if s.net_id == VICTIM]

    buf = io.StringIO()
    with redirect_stdout(buf):
        ok, fail, _t, results_data = batch_route(
            'synthetic', '', ['X'],
            layers=['F.Cu'],
            clearance=0.1, track_width=0.15,
            via_size=0.5, via_drill=0.3, grid_step=0.1,
            ordering_strategy='original',
            rip_existing_nets=['VICTIM'],
            final_reconcile=False,
            return_results=True, pcb_data=pcb)
    out = buf.getvalue()
    sys.stdout.write(out)

    check("X routed through the ripped corridor", ok >= 1)
    x_segs = [s for r in results_data['results']
              for s in (r.get('new_segments') or []) if s.net_id == X]
    check("X has new copper", bool(x_segs))
    check("VICTIM was ripped by the ladder", 'Ripped VICTIM' in out
          or 'Ripping up VICTIM' in out)

    # 1. Casualty custody consumed on the route.py front.
    check("casualties-only reconcile ran",
          'Final reconciliation (casualties-only' in out)

    # 2. The invariant: VICTIM must either carry copper in the write model,
    #    or be loudly owned (coverage gate + failed_multipoint entry).
    victim_new = [s for r in results_data['results']
                  for s in (r.get('new_segments') or []) if s.net_id == VICTIM]
    strip_ids = {id(s) for s in results_data['segments_to_remove']}
    victim_kept_orig = [s for s in victim_orig_seg if id(s) not in strip_ids]
    victim_has_copper = bool(victim_new or victim_kept_orig)
    m = re.findall(r'JSON_SUMMARY: (\{.*\})', out)
    check("JSON_SUMMARY present", bool(m))
    summary = json.loads(m[-1]) if m else {}
    victim_reported = any(e['net_name'] == 'VICTIM'
                          for e in summary.get('failed_multipoint', []))
    check("VICTIM not silently dropped (copper OR owned failure)",
          victim_has_copper or victim_reported)
    if not victim_has_copper:
        check("coverage gate fired for VICTIM", 'COVERAGE GATE: VICTIM' in out)
        check("VICTIM in failed_multipoint (feeds reconciliation)",
              victim_reported)
        check("coverage_gate_nets JSON key lists VICTIM",
              'VICTIM' in summary.get('coverage_gate_nets', []))
        check("casualty_reconcile tally accounts for VICTIM",
              'VICTIM' in (summary.get('casualty_reconcile', {})
                           .get('unrecovered', [])
                           + summary.get('casualty_reconcile', {})
                           .get('partial', [])))

    # 3. DRC safety: any VICTIM copper in the write model must not cross X's
    #    new track (the restore was collision-aware).
    def _crosses(a, b):
        if a.layer != b.layer:
            return False
        def _ccw(p, q, r):
            return (r[1]-p[1])*(q[0]-p[0]) > (q[1]-p[1])*(r[0]-p[0])
        p1, p2 = (a.start_x, a.start_y), (a.end_x, a.end_y)
        p3, p4 = (b.start_x, b.start_y), (b.end_x, b.end_y)
        return (_ccw(p1, p3, p4) != _ccw(p2, p3, p4)
                and _ccw(p1, p2, p3) != _ccw(p1, p2, p4))
    crossing = any(_crosses(vs, xs)
                   for vs in victim_new + victim_kept_orig for xs in x_segs)
    check("no VICTIM copper crossing X (collision-aware custody)",
          not crossing)

    failed = [n for n, okk in CHECKS if not okk]
    print("-" * 60)
    print(f"{len(CHECKS) - len(failed)}/{len(CHECKS)} checks passed")
    if failed:
        print("FAILED: " + ", ".join(failed))
        sys.exit(1)
    print("ALL PASS")


if __name__ == '__main__':
    main()
