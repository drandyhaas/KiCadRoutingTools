#!/usr/bin/env python3
"""#1548: the graze micro-shift must not slide an end that lies inside other
same-net copper, which takes a soft joint out of close_soft_joints' reach.

A Phase 3 tap lands MID-SPAN on a same-net stub, 0.05 mm short of the stub's
free end: a soft joint (connectivity.strict_joint_roots keeps a mid-span
landing flagged on purpose), which close_soft_joints firms with an in-flank
bridge along the stub's own centreline -- no new copper, so no clearance
gate. But the cleanup's micro-shift runs first, and the tap end grazes a
foreign track. Its VERTEX shift slid the end 0.0235 mm off the stub's
centreline. That cleared nothing -- the end sits in the stub's copper, which
grazes the track at least as closely -- but the bridge was no longer
in-flank, and the bridge's clearance gate refused it through that same
graze. The board shipped a check_drc SEGMENT-ENDPOINT-GAP (three on one net
of a stress board, laid in the final reconciliation's own cleanup). A vertex
whose cap lies inside another same-net track no longer slides.

  1. the issue's shape: the end stays on the stub's centreline, close
     bridges the joint in flank, check_drc reports no soft joint;
  2. a mid-span landing with no free end near it is not slid either (the
     slide could not clear the stub's own graze);
  3. control: an end PAST the stub's free end, its cap protruding toward
     the foreign track, is still micro-shifted off the graze.

    python3 tests/test_1548_microshift_keeps_soft_joints.py
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb                 # noqa: E402
from check_drc import run_drc                            # noqa: E402
from routing_config import GridRouteConfig               # noqa: E402
from pcb_modification import (nudge_grazing_microshift,  # noqa: E402
                              close_soft_joints)

FAILS = []


def check(name, cond, detail=''):
    print(("  ok   " if cond else "  FAIL ") + name
          + (f"  {detail}" if detail else ''))
    if not cond:
        FAILS.append(name)


W = 0.0889
TAP = (24.0, 8.0, 24.0, 10.0, W, 'A')            # this run's tap, ends at y=10
# The foreign track 0.1305 mm edge to edge below the tap end: a graze at the
# routed 0.15 clearance (shortfall 0.0195, inside the 0.025 shift cap), clean
# at the 0.1 the board is graded at.
FOREIGN = (23.9, 10.225, 24.1, 10.225, 0.1, 'B')


def board(path, segments):
    pads = [('R1', 24.0, 8.0, 'A'), ('R2', 26.4, 10.0, 'A'),
            ('R3', 21.0, 12.0, 'B')]
    fps = ''.join(f'''
  (footprint "T:P" (layer "F.Cu") (at {x} {y})
    (property "Reference" "{ref}" (at 0 -1.5 0) (layer "F.SilkS"))
    (attr smd)
    (pad "1" smd rect (at 0 0) (size 0.4 0.4) (layers "F.Cu") (net "{n}")))'''
                  for ref, x, y, n in pads)
    segs = ''.join(f'\n  (segment (start {a} {b}) (end {c} {d}) (width {w}) '
                   f'(layer "F.Cu") (net "{n}"))'
                   for a, b, c, d, w, n in segments)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f'''(kicad_pcb (version 20250114) (generator "pcbnew") (generator_version "10.0")
  (general (thickness 1.6))
  (paper "A4")
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (gr_rect (start 10 0) (end 40 20) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts")){fps}{segs}
)
''')


def run(stub, tmp, tag):
    """Board with `stub` (input copper) + the tap (this run's copper) + the
    foreign track; run the micro-shift and close as the cleanup does.
    Returns (tap end y, shifted count, bridges, check_drc soft joints)."""
    src = os.path.join(tmp, f'{tag}_in.kicad_pcb')
    board(src, [stub, TAP, FOREIGN])
    pcb = parse_kicad_pcb(src)
    nets = {n.name: nid for nid, n in pcb.nets.items()}
    tap = next(s for s in pcb.segments
               if (s.start_x, s.start_y) == TAP[:2] and s.net_id == nets['A'])
    results = [{'new_segments': [tap], 'new_vias': []}]
    cfg = GridRouteConfig(clearance=0.15, track_width=0.15, via_size=0.45,
                          via_drill=0.3, layers=['F.Cu', 'B.Cu'],
                          grid_step=0.05)
    with contextlib.redirect_stdout(io.StringIO()):
        shifted = nudge_grazing_microshift(
            results, pcb, None, clearance=0.15, max_shift=0.025,
            keep_input_copper=True, config=cfg)[0]
        bridged = close_soft_joints(results, pcb, None, cfg)
    names = {nid: n.name for nid, n in pcb.nets.items()}
    out = os.path.join(tmp, f'{tag}_out.kicad_pcb')
    board(out, [(s.start_x, s.start_y, s.end_x, s.end_y, s.width,
                 names[s.net_id]) for s in pcb.segments])
    tap_now = [s for s in pcb.segments if s.net_id == nets['A']
               and (s.start_x, s.start_y) == TAP[:2]]
    end_y = tap_now[0].end_y if len(tap_now) == 1 else None
    v = run_drc(out, clearance=0.1, quiet=True, print_summary=False)
    joints = [x for x in v if x.get('type') == 'segment-endpoint-gap']
    return end_y, shifted, bridged, joints


def main():
    with tempfile.TemporaryDirectory() as tmp:
        print("1. the tap lands mid-span, 0.05 mm short of the stub's free end")
        end_y, shifted, bridged, joints = run(
            (23.95, 10.0, 26.4, 10.0, W, 'A'), tmp, 'joint')
        check("the micro-shift leaves the end on the stub's centreline",
              end_y == 10.0 and shifted == 0,
              f"tap end y={end_y}, {shifted} change(s)")
        check("close_soft_joints firms the joint in flank", bridged == 1,
              f"{bridged} bridge(s)")
        check("check_drc reports no soft joint", not joints,
              f"{[(j['loc1'], j['loc2']) for j in joints]}")

        print("2. the tap lands mid-span with no free end near it")
        end_y, shifted, bridged, joints = run(
            (23.5, 10.0, 26.4, 10.0, W, 'A'), tmp, 'tee')
        check("is not slid: the stub's copper grazes there anyway",
              end_y == 10.0 and shifted == 0,
              f"tap end y={end_y}, {shifted} change(s)")
        check("and is no soft joint", not joints and bridged == 0,
              f"{len(joints)} joint(s), {bridged} bridge(s)")

        print("3. control: the tap end lies past the stub's free end")
        end_y, shifted, bridged, joints = run(
            (24.05, 10.0, 26.4, 10.0, W, 'A'), tmp, 'past')
        check("its protruding cap is still micro-shifted off the graze",
              shifted > 0 and end_y is not None and end_y < 10.0,
              f"tap end y={end_y}, {shifted} change(s)")

    if FAILS:
        print(f"\nFAIL: {len(FAILS)} check(s): {FAILS}")
        sys.exit(1)
    print("\nALL PASS")


if __name__ == '__main__':
    main()
