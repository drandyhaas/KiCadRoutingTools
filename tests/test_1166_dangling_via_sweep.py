#!/usr/bin/env python3
"""A via reached on one layer only, and the dead branch it ends, are removed
(#1166).

StickHub VIN shipped a via (KiCad via_dangling, check_weird dangling-via)
ending a 6.8 mm B.Cu spur that tees into the net mid-segment; One-Air-Max
shipped dead branches running VIA TO VIA, so removing one exposes the next.
No pass removed either: the dead-end passes count any same-net via as an
anchor, trim_net_stub_debris skips multipoint nets, and the strict collapse
never makes an input via worse.

Rows, on `sweep_dangling_via_branches` and then through route.py on a board
file (both its passes: the normal end of a run and the "nothing to route"
return):

  * the StickHub shape: via + spur gone, the trunk and the net intact
  * via to via: both go, the second on the next round (a fixed point)
  * a removal that would leave a new dangling end is refused
  * a healthy via, a locked via and protected (input) copper are untouched
  * route.py ships the board without it, and check_weird reads clean

    python3 tests/test_1166_dangling_via_sweep.py
"""
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import Pad, Via, Segment, Net, PCBData, BoardInfo   # noqa: E402
from pcb_modification import sweep_dangling_via_branches              # noqa: E402
from check_connected import check_net_connectivity                    # noqa: E402
from check_weird import check_weird as _cw                              # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def pad(ref, x, y, layer='B.Cu'):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=1.0, size_y=1.0, shape='rect',
               layers=[layer], net_id=1, net_name='VIN', rotation=0.0)


def seg(x1, y1, x2, y2, layer='B.Cu', w=0.2, net=1):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=w,
                   layer=layer, net_id=net)


def via(x, y, net=1, locked=False):
    return Via(x=x, y=y, size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=net,
               locked=locked)


def board(segs, vias, pads):
    return PCBData(footprints={}, nets={1: Net(1, 'VIN')}, segments=list(segs),
                   vias=list(vias), board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu']),
                   pads_by_net={1: list(pads)})


def weird(b):
    b.zones = []
    return _cw(b, quiet=True)[0]


def connected(b):
    return check_net_connectivity(1, b.segments, b.vias, b.pads_by_net[1], [])['connected']


PADS = [pad('A', 10, 10), pad('B', 30, 10)]
TRUNK = seg(10, 10, 30, 10)

print('A. the StickHub shape')
V = via(15, 20)
SPUR = [seg(15, 20, 15, 15), seg(15, 15, 20, 15), seg(20, 15, 20, 10)]   # tees into TRUNK
b = board([TRUNK] + SPUR, [V], PADS)
check('fixture: connected, with a dangling via', connected(b)
      and any(f['category'] == 'dangling-via' for f in weird(b)))
rs, rv = sweep_dangling_via_branches(b, {1})
check('the via and its spur go', [id(x) for x in rv] == [id(V)]
      and {id(x) for x in rs} == {id(x) for x in SPUR}, f'{len(rs)} segs, {len(rv)} vias')
check('the trunk stays and the net is still connected',
      b.segments == [TRUNK] and connected(b))
check('check_weird reads clean', weird(b) == [],
      str(weird(b)))

print('B. via to via: a fixed point')
V1, V2 = via(15, 20), via(25, 20)
CH = [seg(15, 20, 20, 20), seg(20, 20, 25, 20)]
b = board([TRUNK] + CH, [V1, V2], PADS)
rs, rv = sweep_dangling_via_branches(b, {1})
check('both vias and the branch between them go',
      {id(x) for x in rv} == {id(V1), id(V2)} and len(rs) == 2, f'{len(rs)} segs, {len(rv)} vias')
check('...and the board is clean', b.segments == [TRUNK] and not b.vias
      and weird(b) == [])

print('C. the gate refuses a removal that would leave a dangle')
V = via(15, 20)
LINK = seg(15, 20, 25, 20)                 # the via's branch
STUB = seg(20, 20, 20, 10)                 # tees LINK's body to the trunk
b = board([TRUNK, LINK, STUB], [V], PADS)
rs, rv = sweep_dangling_via_branches(b, {1})
check('a branch another track tees into is not taken (nor its via)',
      not rs and not rv and len(b.segments) == 3, f'{len(rs)} segs, {len(rv)} vias')

print('D. what is never touched')
H = via(20, 10)                            # on the trunk (B.Cu) ...
F = seg(20, 10, 20, 0, layer='F.Cu')       # ... and on F.Cu, to a pad there
b = board([TRUNK, F], [H], PADS + [pad('C', 20, 0, layer='F.Cu')])
check('a via joining two layers stays', sweep_dangling_via_branches(b, {1}) == ([], []))
L = via(15, 20, locked=True)
b = board([TRUNK] + SPUR, [L], PADS)
check('a locked dangling via stays', sweep_dangling_via_branches(b, {1}) == ([], []))
V = via(15, 20)
b = board([TRUNK] + SPUR, [V], PADS)
check('protected (input) copper stays',
      sweep_dangling_via_branches(b, {1}, protected_ids={id(V)}) == ([], []))
b = board([TRUNK] + SPUR, [V], PADS)
check('a net outside the scope stays', sweep_dangling_via_branches(b, {2}) == ([], []))
b = board([TRUNK] + SPUR, [V], PADS + [pad('E', 35, 25)])   # E is not connected
check('an unfinished net keeps its copper (#473: the next step welds to it)',
      sweep_dangling_via_branches(b, {1}) == ([], []))

print('E. route.py ships the board without it, on both of its paths')
HEAD = """(kicad_pcb (version 20260206) (generator "pcbnew") (generator_version "10.0")
\t(general (thickness 1.6)) (paper "A4")
\t(layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
\t(setup (pad_to_mask_clearance 0))
\t(net 0 "") (net 1 "VIN") (net 2 "SIG")
\t(gr_rect (start 0 0) (end 40 30) (stroke (width 0.1) (type solid)) (fill none) (layer "Edge.Cuts"))
"""
PADT = """\t(footprint "t:P" (layer "B.Cu") (uuid "aaaaaaaa-0000-0000-0000-0000000{i:05d}") (at {x} {y})
\t\t(property "Reference" "{ref}" (at 0 -2 0) (layer "B.SilkS") (effects (font (size 1 1) (thickness 0.15))))
\t\t(pad "1" smd rect (at 0 0) (size 1 1) (layers "B.Cu" "B.Mask") (net "{net}")
\t\t\t(uuid "cccccccc-0000-0000-0000-0000000{i:05d}")))
"""
SEGT = """\t(segment (start {a} {b}) (end {c} {d}) (width 0.2) (layer "B.Cu") (net "VIN")
\t\t(uuid "bbbbbbbb-0000-0000-0000-0000000{i:05d}"))
"""
VIAT = """\t(via (at 15 20) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net "VIN")
\t\t(uuid "dddddddd-0000-0000-0000-000000000001"))
"""


def write_board(path, with_sig):
    pads = [('A', 10, 10, 'VIN'), ('B', 30, 10, 'VIN')]
    if with_sig:
        pads += [('C', 10, 25, 'SIG'), ('D', 30, 25, 'SIG')]
    segs = [(10, 10, 30, 10), (15, 20, 15, 15), (15, 15, 20, 15), (20, 15, 20, 10)]
    with open(path, 'w', encoding='utf-8') as f:
        f.write(HEAD + ''.join(PADT.format(i=i, ref=r, x=x, y=y, net=n)
                               for i, (r, x, y, n) in enumerate(pads))
                + ''.join(SEGT.format(i=i, a=a, b=bb, c=c, d=d)
                          for i, (a, bb, c, d) in enumerate(segs))
                + VIAT + ')\n')


from kicad_parser import parse_kicad_pcb   # noqa: E402
tmp = tempfile.mkdtemp(prefix='t1166_')
try:
    for tag, with_sig, nets in (('nothing-to-route return', False, ['VIN']),
                                ('normal end of a run', True, ['VIN', 'SIG'])):
        src = os.path.join(tmp, f'in_{with_sig}.kicad_pcb')
        out = os.path.join(tmp, f'out_{with_sig}.kicad_pcb')
        write_board(src, with_sig)
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_router', 'route.py'), src, out,
                            '--nets', *nets, '--track-width', '0.2',
                            '--clearance', '0.2'],
                           capture_output=True, text=True, cwd=ROOT)
        ok = os.path.isfile(out)
        b = parse_kicad_pcb(out) if ok else None
        check(f'{tag}: the sweep ran and says so',
              'Dangling vias (#1166, end of run): removed 1 via' in r.stdout,
              (r.stdout + r.stderr)[-500:])
        check(f'{tag}: no VIN via ships, the trunk does, VIN stays connected',
              ok and not [v for v in b.vias if v.net_id == 1]
              and any(abs(s.start_x - 10) < 1e-6 and abs(s.end_x - 30) < 1e-6
                      for s in b.segments if s.net_id == 1)
              and check_net_connectivity(1, [s for s in b.segments if s.net_id == 1],
                                         [], b.pads_by_net[1], [])['connected'])
        check(f'{tag}: check_weird finds nothing on VIN',
              ok and not [f for f in _cw(b, quiet=True)[0] if f['net'] == 'VIN'],
              str([f for f in _cw(b, quiet=True)[0] if f['net'] == 'VIN']) if ok else '')
    src = os.path.join(tmp, 'in_False.kicad_pcb')
    out = os.path.join(tmp, 'keep.kicad_pcb')
    subprocess.run([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'route.py'),
                    src, out, '--nets', 'VIN', '--track-width', '0.2', '--clearance', '0.2',
                    '--keep-input-copper'], capture_output=True, text=True, cwd=ROOT)
    check('--keep-input-copper: the input via stays',
          os.path.isfile(out) and len(parse_kicad_pcb(out).vias) == 1)
finally:
    shutil.rmtree(tmp, ignore_errors=True)

print()
if FAILS:
    print(f'{len(FAILS)} FAILURE(S): {", ".join(FAILS)}')
    sys.exit(1)
print('ALL PASS')
