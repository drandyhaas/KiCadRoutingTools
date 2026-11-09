#!/usr/bin/env python3
"""#1697: a whole-net retry (the terminal geometry escalation, the
own-requirement retry) passes the same net-tie gate as a rescue rung.

A Kelvin-shunt sense net may touch its net-tie PARTNER pad only where the
contact lies on its own pad (KiCad's per-item waiver). The rescue rungs
reject a route that runs alongside the partner sub-clearance beyond that
waiver (net_rescue._tie_band_violations); the terminal escalation's
per-edge loop (_attempt_net_at_geometry) did not, so a sense net whose
four rescue rungs had just been rejected for exactly that shipped the
same class of copper as "recovered", graded by KiCad as clearance
violations against the partner pad.

On a two-footprint fixture (a shunt R1 whose sense pad sits at its VBUS
pad's edge, tied by net_tie_pad_groups, and the sense net's far pad on R2),
_attempt_net_at_geometry is driven with the search replaced by a route of
known geometry:

  1. A route that leaves the sense pad and runs down the partner pad's side
     sub-clearance is REJECTED: no edge kept, the net not connected, and
     the board carries none of its copper.
  2. A route that leaves the sense pad away from the partner (the contact
     lies on the own pad) is accepted and connects the net.
  3. check_drc grades the rejected route as a pad-segment violation against
     the partner and the accepted one clean, so the gate agrees with the
     grader.

    python3 tests/test_1697_whole_net_tie_gate.py
"""
import contextlib
import io
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

# R1: VBUS pad 1 spans x 9.525..10.475, y 9.925..10.725; SENSE pad 2 spans
# x 9.75..10.25, y 9.925..10.075, on pad 1's top edge. R2.1 is SENSE's far pad.
BOARD = """(kicad_pcb (version 20241229) (generator "pcbnew")
  (general (thickness 1.6))
  (layers
    (0 "F.Cu" signal)
    (2 "B.Cu" signal)
    (25 "Edge.Cuts" user)
  )
  (net 0 "")
  (net 1 "VBUS")
  (net 2 "SENSE")
  (footprint "test:kelvin_shunt" (layer "F.Cu")
    (at 10 10)
    (property "Reference" "R1" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (net_tie_pad_groups "1, 2")
    (pad "1" smd rect (at 0 0.325) (size 0.95 0.8) (layers "F.Cu") (net 1 "VBUS"))
    (pad "2" smd rect (at 0 0) (size 0.5 0.15) (layers "F.Cu") (net 2 "SENSE"))
  )
  (footprint "test:res" (layer "F.Cu")
    (at 9.4 14)
    (property "Reference" "R2" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "F.Cu") (net 2 "SENSE"))
  )
  (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (stroke (width 0.1) (type solid)))
)
"""
WIDTH = 0.1
CLEARANCE = 0.1
# Down the partner's left side 0.025 mm off its edge (x 9.525 - 0.05 - 0.025).
RIDES = [(9.8, 10.0), (9.45, 10.0), (9.45, 14.0)]
# North off the own pad, around the partner at full clearance.
CLEAR = [(10.0, 10.0), (10.0, 9.6), (9.2, 9.6), (9.2, 14.0), (9.4, 14.0)]
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def board(tmp, extra=''):
    path = os.path.join(tmp, 'b.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(BOARD.rstrip()[:-1] + extra + '\n)\n')
    return path


def segments(points, net_id):
    from kicad_parser import Segment
    return [Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                    width=WIDTH, layer='F.Cu', net_id=net_id)
            for a, b in zip(points, points[1:])]


def attempt(tmp, points):
    """_attempt_net_at_geometry on a fresh board with the search returning
    POINTS as the SENSE route: (edge_results, fully, sense segments left)."""
    import net_rescue
    import single_ended_routing
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board(tmp))
    sense = next(i for i, n in pcb.nets.items() if n.name == 'SENSE')
    cfg = GridRouteConfig(grid_step=0.05, clearance=CLEARANCE, track_width=WIDTH,
                          via_size=0.45, via_drill=0.2, layers=['F.Cu', 'B.Cu'])
    real = single_ended_routing.route_net_with_obstacles

    def fake(_pcb, net_id, _cfg, _obs, **_kw):
        return {'net_id': net_id, 'new_segments': segments(points, net_id),
                'new_vias': [], 'iterations': 1}

    single_ended_routing.route_net_with_obstacles = fake
    log = io.StringIO()
    try:
        with contextlib.redirect_stdout(log):
            edges, fully = net_rescue._attempt_net_at_geometry(pcb, sense, cfg, None)
    finally:
        single_ended_routing.route_net_with_obstacles = real
    left = [s for s in pcb.segments if s.net_id == sense]
    return edges, fully, left, log.getvalue()


def drc_rows(tmp, points):
    from check_drc import run_drc
    sexpr = ''.join(
        f'\n  (segment (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) (width {WIDTH}) '
        f'(layer "F.Cu") (net 2) (uuid "aaaaaaaa-0000-0000-0000-00000000000{i}"))'
        for i, (a, b) in enumerate(zip(points, points[1:])))
    with contextlib.redirect_stdout(io.StringIO()):
        v = run_drc(board(tmp, sexpr), clearance=CLEARANCE, quiet=True,
                    print_summary=False, check_sizes=False)
    return [x for x in v if x.get('type') == 'pad-segment'
            and {x.get('net1'), x.get('net2')} == {'VBUS', 'SENSE'}]


def main():
    with tempfile.TemporaryDirectory(prefix='t1697_') as tmp:
        print("1. a route riding the partner pad's side")
        edges, fully, left, log = attempt(tmp, RIDES)
        check("the edge is rejected and the net stays open",
              edges == [] and not fully, f"edges={len(edges)} fully={fully}")
        check("the board keeps none of its copper", left == [], f"{len(left)} segment(s)")
        check("the log names the partner pad",
              'whole-net edge rejected' in log and 'R1.1' in log,
              log.strip().splitlines()[-1] if log.strip() else '')

        print("2. a route leaving the sense pad away from the partner")
        edges, fully, left, _log = attempt(tmp, CLEAR)
        check("the edge is kept and the net connects",
              len(edges) == 1 and fully, f"edges={len(edges)} fully={fully}")

        print("3. check_drc agrees")
        rides, clear = drc_rows(tmp, RIDES), drc_rows(tmp, CLEAR)
        check("the riding route is a pad-segment violation against VBUS",
              len(rides) > 0, f"{len(rides)} row(s)")
        check("the clear route grades clean", clear == [], repr(clear)[:200])
    if failures:
        print(f"\nFAIL: {len(failures)} check(s): {failures}")
        return 1
    print("\nPASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
