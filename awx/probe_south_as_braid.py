#!/usr/bin/env python3
"""Is the "south river" just the braid, rotated?

The emitter special-cases nets whose stub sits below the destination and
routes them with a separate ~87-line builder. If that group is simply a
braid whose flow runs south-to-north, then the special case is
unnecessary: the same braid in a rotated frame should route it.

This prints, for a checkpoint, which nets the south split takes and what
their launch->entry permutation looks like IN A SOUTH-FACING FRAME --
the numbers the braid would actually work on."""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import topo_emit as te  # noqa: E402
import flow_frame as ff  # noqa: E402

K = sys.argv[1] if len(sys.argv) > 1 else '21'
names = subprocess.run([sys.executable,
                        os.path.join(HERE, 'coherent_nets.py'), K],
                       capture_output=True, text=True).stdout.strip()
names = [n for n in names.split(',') if n]
pcb = parse_kicad_pcb(os.path.join(HERE, 'fb_t2q_base.kicad_pcb'))
byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
ends = te.endpoints(pcb, names, byname)
comps = {ends[n][2] for n in names}
pads = [p for c in comps for p in pcb.footprints[c].pads]
rows = sorted({round(p.global_y, 3) for p in pads})

south = [n for n in names if ends[n][0][1] > rows[-1] + 0.8]
west = [n for n in names if n not in south]
print(f'K={K}: {len(west)} west, {len(south)} south -> {south}')
if not south:
    sys.exit(0)

for label, group in (('west', west), ('south', south)):
    if not group:
        continue
    # let the GROUP compute its own flow angle, exactly as the real code
    # would -- guessing it is how the corridor came out backwards
    ang = ff.flow_angle([ends[n][0] for n in group],
                        [ends[n][1] for n in group])
    # in a frame rotated by `ang`, the launch coordinate is the rotated
    # tooth's transverse position and the target is the rotated ball's
    cx = sum(ends[n][1][0] for n in group) / len(group)
    cy = sum(ends[n][1][1] for n in group) / len(group)
    _, back = ff.rotate_pcb(pcb, ang, cx, cy)
    import math
    r = math.radians(ang)
    C, S = math.cos(r), math.sin(r)

    def fwd(x, y):
        dx, dy = x - cx, y - cy
        return (cx + C * dx - S * dy, cy + S * dx + C * dy)

    launch = sorted(group, key=lambda n: fwd(*ends[n][0])[1])
    target = sorted(group, key=lambda n: (fwd(*ends[n][1])[1],
                                          fwd(*ends[n][1])[0]))
    tr = {n: i for i, n in enumerate(target)}
    ranks = [tr[n] for n in launch]
    inv = sum(1 for i in range(len(ranks))
              for j in range(i + 1, len(ranks)) if ranks[i] > ranks[j])
    lis = len(te.lis_keep(ranks))
    us = [fwd(*ends[n][0])[0] for n in group]
    vs = [fwd(*ends[n][0])[1] for n in group]
    bus = [fwd(*ends[n][1])[0] for n in group]
    print(f'\n{label} group: own flow angle {ang:.0f} deg ({len(group)} nets):')
    print(f'   ranks {ranks}')
    print(f'   inversions {inv}  LIS {lis}  via floor 2*(K-LIS) = '
          f'{2 * (len(group) - lis)}')
    print(f'   tooth u span {min(us):.2f}..{max(us):.2f}   '
          f'ball u span {min(bus):.2f}..{max(bus):.2f}')
    print(f'   corridor width (ball_u_min - tooth_u_max) = '
          f'{min(bus) - max(us):.2f} mm'
          + ('   <-- NEGATIVE: teeth are not upstream of the balls'
             if min(bus) - max(us) < 0 else ''))
