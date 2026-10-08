#!/usr/bin/env python3
"""synth_handoff.py -- the whole route's trunk-to-ring HANDOFF on generated cases, in every situation (#622).

  python3 synth_handoff.py [--only TAG,..] [--jobs N] [--outdir DIR] [--list]
  python3 synth_handoff.py --modal APP [--keep-app] ...   (each case in its own container, the app stopped at the end;
                                                           APP deployed first: modal deploy --name APP awx/modal_whole.py)
  python3 synth_handoff.py --collect --outdir DIR          (a --modal run whose client went away)

Each case is a synth_bus.py bus whose destination takes some of its lanes on its north and/or south face (--ring-n /
--ring-s), so the whole route hands them from its trunk to a ring round the destination's corner; with 0402s at that
corner (--hcap): beside the facing column as zynq's C98 stands by U2, just past the corner in the ring's path, two
stacked, on F or on B, either way round; the facing column's lanes packed against the corner or centred
(--w-align); the destination straight across the channel or moved across it, so the bus arrives at an angle
(--dst-dy); the lanes in order or crossing; a pair among them. And the obstacles round them (--part): a passive whose
two pads a lane must pass between, a row of them a lane's gap apart (and one with no gap), a passive square in front
of a berth's or a tooth's stub, PTH header rows across the channel, in the ring's path and in front of the facing
column, mounting holes, rows of via-sized barrels, and all of it at once. make_bench.py prepares it and whole_route.py routes it
(BASE = the bench, DEST = SD1), and each case is graded on the route -- connected, DRC, open nets, vias, from the
route's own WHOLE line -- and on the handoff itself, read off the round that laid the result (its last geometry):

  rings  the classes the solve saw; a case with no ring lane tests nothing here, and FAILS
  gap    the largest jump between two consecutive pieces of any lane: 0 when every join is drawn
  turn   the sharpest turn of any lane, in degrees: a lane stepping past a corner and back turns near 180 (a lane
         turns 90 into a berth's stub)
  hold   what the geometry PAID holding a ring lane's trunk end on its ring's side of the ring's start (`ringside`)
  static what it paid keeping lanes off parts (reported, not graded: a part a lane cannot be kept off goes back to
         the solve as a cut)

A case PASSES when it is connected, DRC-clean, has no open net, has ring lanes, gap 0, turn under 135 and nothing
paid on the hold. The table goes to OUTDIR/handoff.tsv (default tmp/synth_handoff); each case's files to OUTDIR/TAG.
"""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # #937: a research tool (awx), catalogued, shown at no door

import argparse
import concurrent.futures
import glob
import json
import math
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable

# every case: K, then its synth_bus.py arguments (the destination 8 columns wide: 6 balls a ring face)
B = ['--dst-cols', '8']
C98 = ['--hcap', 'sw:1.8:-1.0:F:v']                   # beside the facing column, a ball row above the corner
SW = B + ['--ring-s', '4', '--w-align', 'south']       # 4 lanes to the south face, the facing column's packed south
CASES = [
    ('s4', 12, B + ['--ring-s', '4']),
    ('n4', 12, B + ['--ring-n', '4']),
    ('ns3', 12, B + ['--ring-n', '3', '--ring-s', '3']),
    ('s4_c98F', 12, B + ['--ring-s', '4', '--w-align', 'south'] + C98),
    ('s4_c98B', 12, B + ['--ring-s', '4', '--w-align', 'south', '--hcap', 'sw:1.8:-1.0:B:v']),
    ('s4_c98h', 12, B + ['--ring-s', '4', '--w-align', 'south', '--hcap', 'sw:1.8:-1.0:F:h']),
    ('s4_pathF', 12, B + ['--ring-s', '4', '--w-align', 'south', '--hcap', 'sw:1.0:0.8:F:h']),
    ('s4_pathB', 12, B + ['--ring-s', '4', '--w-align', 'south', '--hcap', 'sw:1.0:0.8:B:h']),
    ('s4_tight', 12, B + ['--ring-s', '4', '--w-align', 'south', '--hcap', 'sw:1.1:-0.5:F:h']),
    ('s4_two', 12, B + ['--ring-s', '4', '--w-align', 'south'] + C98 + ['--hcap', 'sw:1.8:-2.2:F:v']),
    ('n4_c98F', 12, B + ['--ring-n', '4', '--w-align', 'north', '--hcap', 'nw:1.8:-1.0:F:v']),
    ('ns3_caps', 12, B + ['--ring-n', '3', '--ring-s', '3', '--hcap', 'nw:1.8:-1.0:F:v', '--hcap', 'sw:1.8:-1.0:B:v']),
    ('s4_dyS', 12, B + ['--ring-s', '4', '--w-align', 'south', '--dst-dy', '4'] + C98),
    ('s4_dyN', 12, B + ['--ring-s', '4', '--w-align', 'south', '--dst-dy', '-4'] + C98),
    ('s4_blocks', 12, B + ['--ring-s', '4', '--w-align', 'south', '--pattern', 'blocks', '--blocks', '2'] + C98),
    ('s6_full', 12, B + ['--ring-s', '6', '--w-align', 'south'] + C98),
    ('s4_pairs', 12, B + ['--ring-s', '4', '--w-align', 'south', '--pairs', '2'] + C98),
    ('ns5_k20', 20, B + ['--ring-n', '5', '--ring-s', '5', '--hcap', 'nw:1.8:-1.0:F:v', '--hcap', 'sw:1.8:-1.0:F:v']),
    # the ring's lanes leave the source on the far side of the bundle and cross every other lane to reach their ring
    # (in K44 the south ring's lanes ran inside the bundle and ended their trunk inside the ring's start)
    ('s3_rev', 8, B + ['--ring-s', '3', '--pattern', 'reversed'] + C98),
    ('s3_rev_path', 8, B + ['--ring-s', '3', '--pattern', 'reversed', '--hcap', 'sw:1.0:0.8:F:h']),
    ('ns3_rev', 10, B + ['--ring-n', '3', '--ring-s', '3', '--pattern', 'reversed'] + C98),
    ('s4_shuf1', 12, B + ['--ring-s', '4', '--w-align', 'south', '--pattern', 'shuffle', '--seed', '1'] + C98),
    ('s4_shuf2', 12, B + ['--ring-s', '4', '--w-align', 'south', '--pattern', 'shuffle', '--seed', '2', '--hcap',
                          'sw:1.0:0.8:B:h']),
    # two caps inside the hull's margin at the corner, a lane's gap between them: the ring starts outside both, and a
    # lane cutting between them is shorter (K44's DQ12 and DQ1 between C98's pads)
    ('s4_bulge', 12, B + ['--ring-s', '4', '--w-align', 'south', '--pattern', 'blocks', '--blocks', '2',
                          '--hcap', 'sw:0.9:0.6:F:v', '--hcap', 'sw:0.9:1.95:F:v']),
    ('s4_bulgeW', 12, B + ['--ring-s', '4', '--w-align', 'south', '--pattern', 'blocks', '--blocks', '2',
                           '--hcap', 'sw:1.3:-0.3:F:v', '--hcap', 'sw:1.3:-1.65:F:v']),
    # BETWEEN PADS: with the facing column's lanes packed south (K12, ring-s 4), its ball rows stand at y -1.2, -0.4,
    # .. 4.4 from the destination's middle (dw's V). A 0402 on end centred on a row: the row's lane goes straight
    # between its pads (0.42 mm apart), its neighbours' 0.05 mm past them
    ('btw_0402', 12, SW + ['--part', 'c0402@dw:1.0:1.2:F:v']),
    ('btw_0402B', 12, SW + ['--part', 'c0402@dw:1.0:1.2:B:v']),
    ('btw_0603', 12, SW + ['--part', 'c0603@dw:1.2:1.6:F:v']),           # 0.65 mm between its pads: two lanes
    # a row of 0402s across the facing column's lanes, a lane's gap between caps (pitch 1.0: 0.36 mm) -- and none
    # (pitch 0.9: 0.26 mm)
    ('btw_row', 12, SW + ['--part', 'row@dw:1.8:1.6:F:v:n5:p1.0']),
    ('btw_row_tight', 12, SW + ['--part', 'row@dw:1.8:1.6:F:v:n5:p0.9']),
    ('btw_row_ring', 12, SW + ['--part', 'row@ds:2.0:1.2:F:h:n4:p1.0']),  # under the south face, across the ring
    # PADS IN FRONT OF STUBS: an 0402 square in front of a berth (row y 2.0), on its layer and on the other; one in
    # front of a source tooth; a row along the source's face
    ('front_dst', 12, SW + ['--part', 'c0402@dw:1.1:2.0:F:h']),
    ('front_dstB', 12, SW + ['--part', 'c0402@dw:1.1:2.0:B:h']),
    ('front_dst2', 12, SW + ['--part', 'c0402@dw:1.1:2.0:F:h', '--part', 'c0402@dw:1.1:3.6:F:h']),
    ('front_src', 12, SW + ['--part', 'c0402@sf:0.9:0.4:F:v']),
    ('front_src_row', 12, SW + ['--part', 'row@sf:1.5:0.0:F:v:n6:p1.0']),
    # PTH HEADER ROWS (1.7 mm pads, 2.54 pitch: 0.84 mm between pins, two lanes): across the channel, in the ring's
    # path past the corner, in front of the facing column
    ('pth_ch', 12, SW + ['--part', 'pth@ch:6:0:v:n5']),
    ('pth_corner', 12, SW + ['--part', 'pth@sw:1.5:2.2:h:n4']),
    ('pth_dw', 12, SW + ['--part', 'pth@dw:2.6:1.6:v:n3']),
    # HOLES: a 3.2 mm mounting hole mid-channel, a 2.2 at the corner in the ring's path, one under the south face
    ('npth_ch', 12, SW + ['--part', 'npth@ch:6:0']),
    ('npth_corner', 12, SW + ['--part', 'npth@sw:2.0:2.4:d2.2']),
    ('npth_ring', 12, SW + ['--part', 'npth@ds:2.4:2.6:d2.2']),
    # VIA-SIZED BARRELS (0.6 mm, both layers): across the corner, a stitching row under the south face
    ('vias_corner', 12, SW + ['--part', 'vias@sw:1.2:1.2:h:n4:p0.8']),
    ('vias_ds', 12, SW + ['--part', 'vias@ds:0.8:1.3:h:n6:p1.0']),
    # ALL AT ONCE, zynq-like: both rings, the bus at an angle, a cap pair beside each corner, a decoupling row along
    # the facing column, a stitching row under the south face, a hole in the channel
    ('mix', 16, B + ['--ring-n', '3', '--ring-s', '5', '--dst-dy', '3', '--pattern', 'blocks', '--blocks', '2',
                     '--hcap', 'sw:1.8:-1.0:F:v', '--hcap', 'sw:1.8:-2.2:F:v', '--hcap', 'nw:1.8:-1.0:B:v',
                     '--part', 'row@dw:1.4:0:F:v:n4:p1.0', '--part', 'vias@ds:1.0:1.4:h:n5:p1.0',
                     '--part', 'npth@ch:5:-2:d2.2']),
]


# THE ENDS: a part in front of a stub, every way it can stand there. The source's teeth run 0.425 mm east of its
# facing column (sf: U east of it) on rows 0.8 apart, 0.4 off the middle (V 0.4 is SYN06's); the destination's
# berths 0.425 mm west of its facing column (dw: U west), SYN04's on V 2.0, SYN06's on 3.6; its south face's berths
# 0.425 mm down from it (ds: U east of its west end, V down from its face), on the columns at U 1.6, 2.4, 3.2. A
# 0402 lengthwise (h) reaches 0.75 mm either side of its middle, across (v) 0.32; so lengthwise on a row its near
# pad stands G = U - 1.175 past a tooth's or a berth's exit (a case's _gNN: G in hundredths of a mm). G measures the case against the ends model's own front
# (whole_ends.exit_front: the run out of an exit, 1.2 mm, a via's room 0.33 mm off copper): G < 0 the stub's own
# room taken, 0.1 none for a via, 0.35 room for just one at the exit, 0.6 and 1.0 more, 1.25 just in its reach and
# 1.4 just out, 2.5 far
def _g(tag, side, gs, v, more=()):
    return [(f'{tag}_g{round(g * 100):d}'.replace('-', 'm'), 12,
             SW + ['--part', f'c0402@{side}:{1.175 + g:.3f}:{v}:F:h'] + list(more)) for g in gs]


ENDS = (
    # source teeth: the distance, lengthwise on SYN06's row
    _g('tooth', 'sf', (-0.05, 0.1, 0.35, 0.6, 1.0, 1.25, 1.4, 2.5), 0.4)
    # ...the rows: between two (0.0, 0.8), a quarter off (0.2, 0.6); off the router's grid
    + [(f'tooth_v{t_}', 12, SW + ['--part', f'c0402@sf:1.775:{v_}:F:h']) for t_, v_ in
       (('00', 0.0), ('02', 0.2), ('06', 0.6), ('08', 0.8))]
    + [('tooth_odd', 12, SW + ['--part', 'c0402@sf:1.813:0.437:F:h']),
       ('tooth_odd2', 12, SW + ['--part', 'c0402@sf:1.6667:0.3333:F:h'])]
    # ...turned (90: on end, the tooth between its pads)
    + [(f'tooth_r{a_}', 12, SW + ['--part', f'c0402@sf:1.775:0.4:F:h:r{a_}']) for a_ in (15, 30, 45, 60, 90, 135)]
    # ...other parts 0.6 in front: an 0603, a SOT-23 (two pads astride the tooth, the third on its row), and through
    # every layer, so a via is no answer there: a foreign via 0.35 and 0.8 in front, a test point's pin (a hole is the
    # berths': the bench's own fanout of the source runs its escapes into a hole's clearance)
    + [('tooth_0603', 12, SW + ['--part', 'c0603@sf:2.25:0.4:F:h']),
       ('tooth_sot23', 12, SW + ['--part', 'sot23@sf:2.825:0.4:F']),
       ('tooth_via', 12, SW + ['--part', 'vias@sf:1.075:0.4:h:n1']),
       ('tooth_via_far', 12, SW + ['--part', 'vias@sf:1.525:0.4:h:n1']),
       ('tooth_tp', 12, SW + ['--part', 'pth@sf:1.875:0.4:h:n1'])]
    # ...on the other layer; in front of two neighbouring teeth; a row of them turned 45 across the teeth
    + [('tooth_B', 12, SW + ['--part', 'c0402@sf:1.775:0.4:B:h']),
       ('tooth_two', 12, SW + ['--part', 'c0402@sf:1.775:0.4:F:h', '--part', 'c0402@sf:1.775:-0.4:F:h']),
       ('tooth_row45', 12, SW + ['--part', 'row@sf:2.4:0.4:F:v:n4:p1.0:r45'])]
    # the facing column's berths: the distance, lengthwise on SYN04's row; between rows and a quarter off; off the
    # grid; turned; a SOT-23 its single pad to the berth; a via, a hole; on the other layer
    + _g('berth', 'dw', (-0.05, 0.1, 0.35, 0.6, 1.0, 1.4), 2.0)
    + [('berth_v16', 12, SW + ['--part', 'c0402@dw:1.775:1.6:F:h']),
       ('berth_v22', 12, SW + ['--part', 'c0402@dw:1.775:2.2:F:h']),
       ('berth_odd', 12, SW + ['--part', 'c0402@dw:1.813:2.037:F:h'])]
    + [(f'berth_r{a_}', 12, SW + ['--part', f'c0402@dw:1.775:2.0:F:h:r{a_}']) for a_ in (30, 45, 90)]
    + [('berth_sot23', 12, SW + ['--part', 'sot23@dw:2.825:2.0:F']),
       ('berth_via', 12, SW + ['--part', 'vias@dw:1.075:2.0:h:n1']),
       ('berth_hole', 12, SW + ['--part', 'npth@dw:1.425:2.0:d1.0']),
       ('berth_B', 12, SW + ['--part', 'c0402@dw:1.775:2.0:B:h'])]
    # the south face's ring berths (the middle one, U 2.4): an 0402 along the stub (G = V - 1.175), across it (G = V
    # - 0.745), turned, a via
    + [(f'ring_g{round(g * 100):d}', 12, SW + ['--part', f'c0402@ds:2.4:{1.175 + g:.3f}:F:v'])
       for g in (0.1, 0.35, 0.6, 1.0)]
    + [('ring_across', 12, SW + ['--part', 'c0402@ds:2.4:1.345:F:h']),
       ('ring_r45', 12, SW + ['--part', 'c0402@ds:2.4:1.775:F:v:r45']),
       ('ring_via', 12, SW + ['--part', 'vias@ds:2.4:1.075:h:n1'])]
    # both ends of one lane (SYN06: its tooth on 0.4, its berth on 3.6); the lanes crossing (reversed) with a part in
    # front of a tooth and a berth
    + [('ends_both', 12, SW + ['--part', 'c0402@sf:1.775:0.4:F:h', '--part', 'c0402@dw:1.775:3.6:F:h']),
       ('ends_rev', 8, B + ['--ring-s', '3', '--pattern', 'reversed', '--part', 'c0402@sf:1.775:0.4:F:h',
                            '--part', 'c0402@dw:1.775:0.4:F:h'])]
)
CASES += ENDS

# WALLS: a row of 0402s lengthwise, one on each stub's row, 0.8 apart -- 0.16 mm between caps, so no lane threads it:
# each lane goes to the other layer before it or round its end. In front of the source's eight middle teeth (sf, V 0:
# rows -2.8 .. 2.8) at G = U - 1.175 past their exits; in front of the facing column's berths (dw, V 1.6: rows -1.2 ..
# 4.4); over four teeth alone; on the other layer; a stitching row of foreign vias (no layer answers it: round its
# end); at both ends; with the lanes crossing
WALLS = (
    [(f'wall_src_g{round(g * 100)}', 12, SW + ['--part', f'row@sf:{1.175 + g:.3f}:0.0:F:v:n8:p0.8'])
     for g in (0.1, 0.35, 0.6, 1.0)]
    + [('wall_dst_g60', 12, SW + ['--part', 'row@dw:1.775:1.6:F:v:n8:p0.8']),
       ('wall_src_part', 12, SW + ['--part', 'row@sf:1.775:0.8:F:v:n4:p0.8']),
       ('wall_src_B', 12, SW + ['--part', 'row@sf:1.775:0.0:B:v:n8:p0.8']),
       ('wall_src_vias', 12, SW + ['--part', 'vias@sf:1.525:0.4:v:n4:p0.8']),
       ('wall_both', 12, SW + ['--part', 'row@sf:1.775:0.0:F:v:n8:p0.8', '--part', 'row@dw:1.775:1.6:F:v:n8:p0.8']),
       ('wall_rev', 8, B + ['--ring-s', '3', '--pattern', 'reversed', '--part', 'row@sf:1.775:0.0:F:v:n6:p0.8'])]
)
CASES += WALLS

# WALLS IN THE CHANNEL, past the ends model's reach (the channel runs x 1.2 .. 13.2, the board y -11.2 .. 11.2, the arrays
# y -5.2 .. 5.2): the same row 2.5 mm past the teeth, mid-channel, nearly the board's height (no way round its ends), on
# B, one on F and one on B further on (two changes a lane), 2.5 mm short of the berths, off the bus's middle, a
# stitching row of vias (no layer answers it: round its ends), turned 45 degrees
CHANNEL = (
    [('chan_g250', 12, SW + ['--part', 'row@sf:3.675:0.0:F:v:n8:p0.8']),
     ('chan_mid', 12, SW + ['--part', 'row@ch:6:0.0:F:v:n8:p0.8']),
     ('chan_full', 12, SW + ['--part', 'row@ch:6:0.0:F:v:n24:p0.8']),
     ('chan_B', 12, SW + ['--part', 'row@ch:6:0.0:B:v:n8:p0.8']),
     ('chan_two', 12, SW + ['--part', 'row@ch:4:0.0:F:v:n8:p0.8', '--part', 'row@ch:8:0.0:B:v:n8:p0.8']),
     ('chan_dst_g250', 12, SW + ['--part', 'row@dw:3.675:1.6:F:v:n8:p0.8']),
     ('chan_off', 12, SW + ['--part', 'row@ch:6:2.0:F:v:n6:p0.8']),
     ('chan_vias', 12, SW + ['--part', 'vias@ch:6:0.0:v:n8:p0.8']),
     ('chan_r45', 12, SW + ['--part', 'row@ch:6:0.0:F:v:n8:p0.8:r45'])]
)
CASES += CHANNEL


def run(argv, log, env=None, timeout=None):
    with open(log, 'w') as fh:
        try:
            return subprocess.run(argv, stdout=fh, stderr=subprocess.STDOUT, cwd=HERE, env=env, timeout=timeout).returncode
        except subprocess.TimeoutExpired:
            fh.write(f'\nTIMEOUT after {timeout} s\n')
            return -1


def handoff_marks(geo):
    """(largest jump between consecutive pieces of a lane, sharpest turn in degrees) over every lane of a geometry"""
    gap, turn = 0.0, 0.0
    for L in geo['lanes'].values():
        # (a piece shorter than a nanometre, a board's own unit, is the LP's float noise -- a via at a stub's end)
        pcs = [p for p in L['pieces'] if math.hypot(p[2] - p[0], p[3] - p[1]) > 1e-6]
        for a, b in zip(pcs, pcs[1:]):
            gap = max(gap, math.hypot(b[0] - a[2], b[1] - a[3]))
            d1, d2 = (a[2] - a[0], a[3] - a[1]), (b[2] - b[0], b[3] - b[1])
            c = (d1[0] * d2[0] + d1[1] * d2[1]) / (math.hypot(*d1) * math.hypot(*d2))
            turn = max(turn, math.degrees(math.acos(max(-1.0, min(1.0, c)))))
    return gap, turn


def one(tag, K, args, outdir, timeout, copper=2, fanout_layers=None):
    d = os.path.join(outdir, tag)
    args = list(args) + (['--copper', str(copper)] if copper != 2 else [])
    os.makedirs(d, exist_ok=True)
    raw, bench = os.path.join(d, 'raw.kicad_pcb'), os.path.join(d, 'bench.kicad_pcb')
    row = {'tag': tag, 'k': K, 'args': ' '.join(args)}
    if run([PY, 'synth_bus.py', raw, '--k', str(K)] + args, os.path.join(d, 'gen.log')):
        return dict(row, verdict='GEN FAILED')
    if run([PY, 'make_bench.py', raw, 'SU1', 'SD1', bench] + (['--fanout-layers', fanout_layers] if fanout_layers else []),
           os.path.join(d, 'bench.log')) or not os.path.isfile(bench):
        return dict(row, verdict='BENCH FAILED')
    t0 = time.time()
    rc = run([PY, 'whole_route.py', str(K), os.path.join(d, 'run')], os.path.join(d, 'run.log'),
             env=dict(os.environ, BASE=bench, DEST='SD1'), timeout=timeout)
    row['secs'] = round(time.time() - t0)
    whole = [ln for ln in open(os.path.join(d, 'run.log')) if ln.startswith('WHOLE')]
    if not whole:
        return dict(row, verdict=f'NO RESULT (rc {rc})')
    w = dict(re.findall(r'(\w+)=(\S+)', whole[-1]))
    row.update({k: w.get(k) for k in ('round', 'lanes', 'vias', 'connected', 'drc', 'open')})
    cl = [ln for ln in open(os.path.join(d, 'run', 'r1', 'solve.log')) if ln.startswith('classes:')] \
        if os.path.isfile(os.path.join(d, 'run', 'r1', 'solve.log')) else []
    row['rings'] = re.sub(r'\s+', '', cl[0].split('}')[0].split(':', 1)[1] + '}') if cl else '?'
    gs = sorted(glob.glob(os.path.join(d, 'run', f'r{w.get("round", 1)}', 'loop', 'g*.json')),
                key=lambda p: int(re.sub(r'\D', '', os.path.basename(p)) or 0))
    if gs:
        gap, turn = handoff_marks(json.load(open(gs[-1])))
        paid = open(gs[-1].replace('.json', '.log')).read() if os.path.isfile(gs[-1].replace('.json', '.log')) else ''
        m = re.search(r'PAID ringside (\d+)', paid)
        st = re.search(r'PAID static (\d+)', paid)
        row.update(gap=round(gap, 3), turn=round(turn, 1), hold=int(m.group(1)) if m else 0,
                   static=int(st.group(1)) if st else 0)
    ok = (w.get('connected') == '1' and w.get('drc') == '1' and w.get('open') == '0' and ("'S'" in row['rings']
          or "'N'" in row['rings']) and row.get('gap', 1) < 1e-6 and row.get('turn', 180) < 135 and row.get('hold', 1) == 0)
    return dict(row, verdict='PASS' if ok else 'FAIL')


COLS = ('tag', 'k', 'verdict', 'rings', 'round', 'lanes', 'vias', 'connected', 'drc', 'open', 'gap', 'turn', 'hold',
        'static', 'secs', 'args')


def modal_rows(a, cases):
    """--modal: each case spawned on the deployed app's run_synth (modal_whole.py), the call ids kept in
    OUTDIR/calls.json, every case's files unpacked into OUTDIR/TAG as it finishes; --collect: the same from the kept
    calls. The rows once every case is in -- and then the app stopped, its work done (--keep-app keeps it) -- else None
    (the rest still running: --collect again)"""
    import io
    import tarfile
    import modal
    cf = os.path.join(a.outdir, 'calls.json')
    if a.modal:
        fn = modal.Function.from_name(a.modal, 'run_synth')
        calls = {tag: fn.spawn(tag).object_id for tag, _K, _args in cases}
        json.dump({'app': a.modal, 'calls': calls}, open(cf, 'w'), indent=1)
        print(f'{len(calls)} cases spawned on {a.modal}; calls in {cf}', flush=True)
    kept = json.load(open(cf))
    calls, app = (kept['calls'], kept.get('app')) if 'calls' in kept else (kept, None)
    rows, wait = {}, time.time()
    while True:
        for tag, cid in calls.items():
            if tag in rows:
                continue
            try:
                res = modal.FunctionCall.from_id(cid).get(timeout=0)
            except TimeoutError:
                continue
            with tarfile.open(fileobj=io.BytesIO(res['tgz']), mode='r:gz') as t:
                t.extractall(a.outdir, filter='data')
            open(os.path.join(a.outdir, f'{tag}.modal.log'), 'w').write(res['log'])
            r = res['row'] or {'tag': tag, 'verdict': f'NO ROW (rc {res["rc"]})'}
            rows[tag] = r
            print('  '.join(f'{c}={r.get(c)}' for c in COLS if c != 'args'), flush=True)
        if len(rows) == len(calls):
            if app and not a.keep_app:
                r = subprocess.run([sys.executable, '-m', 'modal', 'app', 'stop', '-y', app], capture_output=True,
                                   text=True)
                print(f'every case in: app {app} ' + ('stopped' if r.returncode == 0 else
                                                      f'NOT stopped (rc {r.returncode}): {r.stderr.strip()[-200:]}'),
                      flush=True)
            return list(rows.values())
        if a.collect and not a.modal:
            print(f'{len(rows)}/{len(calls)} in; the rest still running -- --collect again')
            return None
        if time.time() - wait > a.timeout * 3:
            print(f'{len(rows)}/{len(calls)} in after {round(time.time() - wait)} s; --collect later')
            return None
        time.sleep(15)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--only', help='comma-separated case tags')
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--outdir', default=os.path.join(HERE, 'tmp', 'synth_handoff'))
    ap.add_argument('--timeout', type=int, default=1800, help='each whole route, s')
    ap.add_argument('--list', action='store_true', help='print the cases and stop')
    ap.add_argument('--fanout-layers', help='the bench\'s source fanout layers (make_bench --fanout-layers; default '
                    'the routing layers)')
    ap.add_argument('--copper', type=int, default=2, choices=(2, 4), help='each board\'s copper layers (synth_bus '
                    '--copper): 4 for the routing layers ROUTE_LAYERS names from the environment, as F.Cu,B.Cu,In2.Cu')
    ap.add_argument('--modal', metavar='APP', help='run each case in its own container on Modal (up to 50 at once), '
                    'on the app deployed from this tree (modal deploy --name APP awx/modal_whole.py); the calls are '
                    'kept in OUTDIR/calls.json')
    ap.add_argument('--collect', action='store_true', help='gather a --modal run from OUTDIR/calls.json (its client '
                    'gone): every finished case, the table once all are in')
    ap.add_argument('--keep-app', action='store_true', help='--modal / --collect: leave the app deployed once every '
                    'case is in (it is stopped by default)')
    a = ap.parse_args(argv)
    cases = [c for c in CASES if not a.only or c[0] in a.only.split(',')]
    if a.list:
        for tag, K, args in cases:
            print(f'{tag:10s} K={K:2d} {" ".join(args)}')
        return 0
    os.makedirs(a.outdir, exist_ok=True)
    if (a.copper != 2 or a.fanout_layers) and (a.modal or a.collect):
        raise SystemExit('synth_handoff: --copper runs here only (the Modal app builds its boards with two)')
    if a.modal or a.collect:
        rows = modal_rows(a, cases)
        if rows is None:
            return 2
    else:
        rows = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=a.jobs) as ex:
            futs = {ex.submit(one, tag, K, args, os.path.abspath(a.outdir), a.timeout, a.copper,
                                a.fanout_layers): tag for tag, K, args in cases}
            for f in concurrent.futures.as_completed(futs):
                r = f.result()
                rows.append(r)
                print('  '.join(f'{c}={r.get(c)}' for c in COLS if c != 'args'), flush=True)
    order = {c[0]: i for i, c in enumerate(CASES)}
    rows.sort(key=lambda r: order[r['tag']])
    with open(os.path.join(a.outdir, 'handoff.tsv'), 'w') as fh:
        fh.write('\t'.join(COLS) + '\n')
        for r in rows:
            fh.write('\t'.join(str(r.get(c, '')) for c in COLS) + '\n')
    n_pass = sum(r['verdict'] == 'PASS' for r in rows)
    print(f'\n{n_pass}/{len(rows)} PASS -- {os.path.join(a.outdir, "handoff.tsv")}')
    return 0 if n_pass == len(rows) else 1


if __name__ == '__main__':
    sys.exit(main())
