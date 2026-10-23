#!/usr/bin/env python3
"""The bus step's pulls in the escape fanout (route_bus.attach_pulls; coherent_nets.admissible's BUS_ATTACHED).

  python3 tests/test_622_attach_pulls.py

On a board of its own -- a 5 x 5 ball array U1 at 0.8 mm pitch on F.Cu, each tested ball's dog-bone via on its gap
point, a second part U2 on each tested net, and two-pad pull resistors on B.Cu whose pull pads the step has taken off
their nets (route_bus.unpull) -- at the zynq LVDS run's sizes (clearance 0.09, escape track 0.12, via 0.45), this
pins:

1. a pull pad 1.0 mm straight from its ball's via (R1, as the human ties zynq TX_FRAME_N into RX10) is put back on its
   net and joined to the via by one B.Cu stub of the escape track;
2. one whose straight stub another net's track crosses (R3) is joined the short way round along the array's ball and
   gap lines, every piece clear of that track, within ATTACH_PITCHES of the pitch;
3. one farther than that from its via (R2), and one walled off by another net's via (R4), stay off their nets for the
   rest of the chain, no copper laid for them;
4. coherent_nets.admissible: the net with a pull pad joined (three pads on three parts) is a lane only while that pad
   is named in BUS_ATTACHED -- none of its two ends.
"""
import contextlib
import io
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('AWX_RULES', None)

import rules as _rules  # noqa: E402
_rules.install(_rules.Rules(clearance=0.09, track=0.15, via_size=0.45, via_drill=0.3, fan_track=0.12,
                            hole_to_hole=0.25, source='test'))
import awx_settings  # noqa: E402
import route_bus as rb  # noqa: E402
from coherent_nets import admissible  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

N, PITCH = 5, 0.8
# (net index: its ball's dog-bone via; its pull resistor; the pull pad's centre; the resistor's rail pad's)
CASES = {12: ((2.0, 2.0), 'R1', (2.0, 3.0), (2.0, 3.85)),      # straight down its gap column, 1.0 mm
         2: ((1.2, 0.4), 'R2', (-1.0, 0.4), (-1.85, 0.4)),     # 2.2 mm off: past the reach
         6: ((0.4, 0.4), 'R3', (0.8, 1.2), (0.8, 2.05)),       # 0.89 mm, its straight crossed by OTHER's track
         4: ((2.8, 0.4), 'R4', (2.8, 1.2), (3.65, 1.2))}       # 0.8 mm, OTHER's via at (2.8, 0.8) between


def board():
    nets = '\n'.join(f'  (net {i + 1} "N{i}")' for i in range(N * N))
    other, vtt = N * N + 1, N * N + 2
    balls = '\n'.join(f'    (pad "{i}" smd circle (at {(i % N) * PITCH:.3f} {(i // N) * PITCH:.3f}) (size 0.4 0.4) '
                      f'(layers "F.Cu" "F.Mask") (net {i + 1} "N{i}"))' for i in range(N * N))
    u2 = '\n'.join(f'    (pad "{k + 1}" smd circle (at {k * 1.0:.3f} 0) (size 0.4 0.4) (layers "F.Cu" "F.Mask") '
                   f'(net {i + 1} "N{i}"))' for k, i in enumerate(sorted(CASES)))
    parts = ''
    for i, (_via, ref, (x, y), (x2, y2)) in sorted(CASES.items()):
        parts += f"""  (footprint "R" (layer "B.Cu") (at 0 0)
    (property "Reference" "{ref}" (at 0 0) (layer "B.SilkS"))
    (pad "1" smd rect (at {x:.3f} {y:.3f}) (size 0.5 0.5) (layers "B.Cu" "B.Mask") (net {i + 1} "N{i}"))
    (pad "2" smd rect (at {x2:.3f} {y2:.3f}) (size 0.5 0.5) (layers "B.Cu" "B.Mask") (net {vtt} "VTT"))
  )
"""
    vias = ''.join(f'  (via (at {vx:.3f} {vy:.3f}) (size 0.45) (drill 0.3) (layers "F.Cu" "B.Cu") (net {i + 1}))\n'
                   for i, ((vx, vy), _r, _p, _p2) in sorted(CASES.items()))
    vias += f'  (via (at 2.800 0.800) (size 0.45) (drill 0.3) (layers "F.Cu" "B.Cu") (net {other}))\n'
    segs = f'  (segment (start 0.450 0.950) (end 0.550 0.950) (width 0.12) (layer "B.Cu") (net {other}))\n'
    return f"""(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
{nets}
  (net {other} "OTHER")
  (net {vtt} "VTT")
  (footprint "BGA" (layer "F.Cu") (at 0 0)
    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))
{balls}
  )
  (footprint "U2" (layer "F.Cu") (at 20 0)
    (property "Reference" "U2" (at 0 0) (layer "F.SilkS"))
{u2}
  )
{parts}{vias}{segs})
"""


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(path)


fails = []


def check(cond, msg):
    if not cond:
        fails.append(msg)
        print('FAIL', msg)


def _pt_seg(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 < 1e-18 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


td = tempfile.mkdtemp()
path = os.path.join(td, 'fanned.kicad_pcb')
text = board()
src = os.path.join(td, 'src.kicad_pcb')
with open(src, 'w') as f:
    f.write(text)
pcb0 = parse(src)
pads = [next(q for q in pcb0.footprints[ref].pads if q.pad_number == '1') for _v, ref, _p, _p2 in CASES.values()]
text, held = rb.unpull(text, pads)                       # (the step's own: the pull pads off their nets)
with open(path, 'w') as f:
    f.write(text)
check(len(held) == 4, f'0: the four pull pads taken off their nets: {[h[:2] for h in held]}')

log = []
left, attached = rb.attach_pulls(path, held, ['U1', 'U2'], log=log.append)
got = {h[0] for h in attached}
check(got == {'R1', 'R3'}, f'1-3: R1 and R3 joined, R2 and R4 not: joined {sorted(got)}; {log}')
check({h[0] for h in left} == {'R2', 'R4'}, f'3: R2 and R4 left for the chain: {[h[:2] for h in left]}')
pcb = parse(path)
byname = {n.name: i for i, n in pcb.nets.items()}
lim = rb.ATTACH_PITCHES * PITCH
for i, (via, ref, pad, _p2) in sorted(CASES.items()):
    p = next(q for q in pcb.footprints[ref].pads if q.pad_number == '1')
    segs = [s for s in pcb.segments if s.net_id == byname[f'N{i}']]
    if ref in got:
        check(p.net_id == byname[f'N{i}'], f'1: {ref}.1 back on N{i}: net {p.net_id}')
        check(segs and all(s.layer == 'B.Cu' and abs(s.width - 0.12) < 1e-6 for s in segs),
              f'1: {ref}\'s stub on B.Cu at the escape track: {[(s.layer, s.width) for s in segs]}')
        ends = [(round(s.start_x, 3), round(s.start_y, 3)) for s in segs] + \
            [(round(s.end_x, 3), round(s.end_y, 3)) for s in segs]
        check(tuple(via) in ends and tuple(pad) in ends, f'1: {ref}\'s stub from the via {via} to the pad {pad}: {ends}')
        length = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y) for s in segs)
        check(length <= lim + 1e-6, f'2: {ref}\'s stub within {lim:.2f} mm: {length:.3f}')
    else:
        check(not p.net_id, f'3: {ref}.1 still off its net: net {p.net_id}')
        check(not segs, f'3: no copper laid for {ref}: {len(segs)} segment(s)')
r1 = [s for s in pcb.segments if s.net_id == byname['N12']]
check(len(r1) == 1, f'1: R1 joined by one straight stub: {len(r1)} piece(s)')
r3 = [s for s in pcb.segments if s.net_id == byname['N6']]
check(len(r3) >= 2, f'2: R3 joined the way round: {len(r3)} piece(s)')
oth = [s for s in pcb.segments if s.net_id == byname['OTHER']][0]
gap = min(min(_pt_seg((oth.start_x, oth.start_y), (s.start_x, s.start_y), (s.end_x, s.end_y)),
              _pt_seg((oth.end_x, oth.end_y), (s.start_x, s.start_y), (s.end_x, s.end_y))) for s in r3)
check(gap >= 0.06 + 0.06 + 0.09 - 1e-6, f'2: R3\'s stub clear of OTHER\'s track: {gap:.3f}')

# 4. the ladder: N12 has three pads (U1, U2, R1.1) on three parts once joined
net12 = pcb.nets[byname['N12']]
check(len(net12.pads) == 3, f'4: N12 three pads once R1 is joined: {len(net12.pads)}')
check(not admissible(pcb, net12), '4: a three-part net is no lane without BUS_ATTACHED')
with awx_settings.given({**awx_settings.environ(), 'BUS_ATTACHED': 'R1.1'}):
    check(admissible(pcb, net12), '4: with R1.1 named in BUS_ATTACHED, N12 is a lane U1 to U2')

print('ALL PASS' if not fails else f'{len(fails)} FAILED')
sys.exit(1 if fails else 0)
