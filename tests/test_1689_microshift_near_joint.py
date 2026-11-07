#!/usr/bin/env python3
"""#1689: the graze micro-shift must not widen a near-coincident joint into a
soft joint check_drc counts.

Two free ends of one net sit 0.0085 mm apart, under the 0.02 mm the pipeline
treats as coincident, and a foreign track grazes both at this run's clearance.
Sliding one end by its shortfall opens the joint to about 0.03 mm. It clears
nothing, because the other end still grazes, and close_soft_joints' clearance
gate then refuses the bridge through that same graze, so the board ships a
SEGMENT-ENDPOINT-GAP the input did not have. KiCad reports nothing there: the
two caps still overlap and the net is connected.

  1. the case: the end stays put, the joint stays under the tolerance, and
     check_drc counts no soft joint;
  2. control: an end with no other free end near it is still slid off the graze;
  3. control: a soft joint already counted in the input does not refuse the
     shift, which adds no joint.

    python3 tests/test_1689_microshift_near_joint.py
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


W = 0.1
TAP = (24.0, 8.0, 24.0, 10.0, W, 'A')            # this run's copper, ends at y=10
# Edge 0.1817 from the tap end: required 0.15 + W/2 = 0.2, so shortfall 0.0183,
# inside the shift cap. Clean at the 0.1 the board is graded at.
FOREIGN = (23.0, 10.2317, 27.0, 10.2317, 0.1, 'B')


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
    """`stub` (input copper) + the tap (this run's copper) + the foreign track;
    the micro-shift and close as the cleanup runs them. Returns the tap end's y,
    the change count and check_drc's counted soft joints."""
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
            results, pcb, None, clearance=0.15, max_shift=0.05,
            keep_input_copper=True, config=cfg)[0]
        close_soft_joints(results, pcb, None, cfg)
    names = {nid: n.name for nid, n in pcb.nets.items()}
    out = os.path.join(tmp, f'{tag}_out.kicad_pcb')
    board(out, [(s.start_x, s.start_y, s.end_x, s.end_y, s.width,
                 names[s.net_id]) for s in pcb.segments])
    now = [s for s in pcb.segments if s.net_id == nets['A']
           and (s.start_x, s.start_y) == TAP[:2]]
    end_y = now[0].end_y if len(now) == 1 else None
    v = run_drc(out, clearance=0.1, quiet=True, print_summary=False)
    return end_y, shifted, [x for x in v if x.get('type') == 'segment-endpoint-gap']


def main():
    with tempfile.TemporaryDirectory() as tmp:
        print("1. two free ends 0.0085 mm apart, both grazing")
        end_y, shifted, joints = run((24.006, 10.006, 26.4, 10.006, W, 'A'), tmp, 'near')
        check("the end is not slid off the joint", end_y == 10.0 and shifted == 0,
              f"tap end y={end_y}, {shifted} change(s)")
        check("check_drc counts no soft joint", not joints,
              f"{[(j['loc1'], j['loc2'], round(j['gap_mm'], 4)) for j in joints]}")

        print("2. control: no other free end near the tap end")
        end_y, shifted, joints = run((24.8, 10.6, 26.4, 10.6, W, 'A'), tmp, 'lone')
        check("the end is still slid off the graze",
              shifted > 0 and end_y is not None and end_y < 10.0,
              f"tap end y={end_y}, {shifted} change(s)")

        print("3. control: the joint is already a counted soft joint in the input")
        stub = (24.0, 9.95, 26.4, 9.95, W, 'A')
        src = os.path.join(tmp, 'soft_input.kicad_pcb')
        board(src, [stub, TAP, FOREIGN])
        before = [x for x in run_drc(src, clearance=0.1, quiet=True, print_summary=False)
                  if x.get('type') == 'segment-endpoint-gap']
        check("the input counts one soft joint", len(before) == 1, f"{len(before)}")
        end_y, shifted, joints = run(stub, tmp, 'soft')
        check("the shift, which adds no joint, is not refused",
              shifted > 0 and end_y is not None and end_y < 10.0,
              f"tap end y={end_y}, {shifted} change(s)")

    if FAILS:
        print(f"FAILED: {len(FAILS)}")
        return 1
    print("PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
